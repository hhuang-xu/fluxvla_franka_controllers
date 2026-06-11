#!/usr/bin/env python3
"""Replay CSV targets to joint or Cartesian Franka controllers."""

import argparse
import csv
import math
from pathlib import Path


JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
LEFT_Q = [f"left_q{i}" for i in range(1, 8)]
RIGHT_Q = [f"right_q{i}" for i in range(1, 8)]
LEFT_POSE = ["left_x", "left_y", "left_z", "left_qx", "left_qy", "left_qz", "left_qw"]
RIGHT_POSE = [
    "right_x",
    "right_y",
    "right_z",
    "right_qx",
    "right_qy",
    "right_qz",
    "right_qw",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Replay CSV targets to joint impedance or Cartesian impedance controllers."
    )
    parser.add_argument("--csv", required=True, help="CSV with qpos or eepose columns")
    parser.add_argument("--control_mode", choices=("joint", "eepose"), default="joint")
    parser.add_argument("--execute", action="store_true", help="publish to ROS")
    parser.add_argument("--replay_hz", type=float, default=30.0)
    parser.add_argument(
        "--left_topic",
        default="/left_arm/ruckig_joint_impedance_controller/target_joint_state",
    )
    parser.add_argument(
        "--right_topic",
        default="/right_arm/ruckig_joint_impedance_controller/target_joint_state",
    )
    parser.add_argument(
        "--left_pose_topic",
        default="/left_arm/cartesian_impedance_controller/equilibrium_pose",
    )
    parser.add_argument(
        "--right_pose_topic",
        default="/right_arm/cartesian_impedance_controller/equilibrium_pose",
    )
    parser.add_argument("--left_frame_id", default="left_arm/panda_link0")
    parser.add_argument("--right_frame_id", default="right_arm/panda_link0")
    parser.add_argument(
        "--left_controller_manager",
        default="/left_arm/controller_manager/list_controllers",
    )
    parser.add_argument(
        "--right_controller_manager",
        default="/right_arm/controller_manager/list_controllers",
    )
    parser.add_argument("--left_joint_controller", default="ruckig_joint_impedance_controller")
    parser.add_argument("--right_joint_controller", default="ruckig_joint_impedance_controller")
    parser.add_argument("--left_eepose_controller", default="cartesian_impedance_controller")
    parser.add_argument("--right_eepose_controller", default="cartesian_impedance_controller")
    parser.add_argument("--skip_controller_check", action="store_true")
    parser.add_argument("--left_gripper_topic", default="/left_arm/franka_gripper/move/goal")
    parser.add_argument(
        "--right_gripper_topic", default="/right_arm/franka_gripper/move/goal"
    )
    parser.add_argument("--no_gripper", action="store_true")
    parser.add_argument("--gripper_speed", type=float, default=0.8)
    return parser.parse_args()


def require_columns(fieldnames, columns):
    missing = [name for name in columns if name not in fieldnames]
    if missing:
        raise RuntimeError(f"CSV missing columns: {', '.join(missing)}")


def read_float(row, name):
    value = row.get(name, "")
    if value == "":
        return None
    return float(value)


def normalize_pose(values):
    pose = [float(v) for v in values]
    norm = math.sqrt(sum(v * v for v in pose[3:7]))
    if norm < 1e-8:
        raise RuntimeError(f"invalid quaternion: {pose[3:7]}")
    pose[3:7] = [v / norm for v in pose[3:7]]
    return pose


def read_csv(path, control_mode):
    path = Path(path).expanduser()
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        if control_mode == "joint":
            require_columns(fieldnames, LEFT_Q + RIGHT_Q)
        else:
            require_columns(fieldnames, LEFT_POSE + RIGHT_POSE)
        rows = []
        for row in reader:
            if control_mode == "joint":
                left = [float(row[name]) for name in LEFT_Q]
                right = [float(row[name]) for name in RIGHT_Q]
            else:
                left = normalize_pose([row[name] for name in LEFT_POSE])
                right = normalize_pose([row[name] for name in RIGHT_POSE])
            rows.append({
                "left": left,
                "right": right,
                "left_gripper": read_float(row, "left_gripper"),
                "right_gripper": read_float(row, "right_gripper"),
            })
    if not rows:
        raise RuntimeError(f"CSV has no data rows: {path}")
    return rows


def make_joint_state(rospy, JointState, q):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = JOINT_NAMES
    msg.position = q
    return msg


def make_pose_stamped(rospy, PoseStamped, pose, frame_id):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.pose.position.x = pose[0]
    msg.pose.position.y = pose[1]
    msg.pose.position.z = pose[2]
    msg.pose.orientation.x = pose[3]
    msg.pose.orientation.y = pose[4]
    msg.pose.orientation.z = pose[5]
    msg.pose.orientation.w = pose[6]
    return msg


def make_gripper_goal(rospy, MoveActionGoal, width, speed, side, seq):
    msg = MoveActionGoal()
    msg.header.stamp = rospy.Time.now()
    msg.goal_id.stamp = msg.header.stamp
    msg.goal_id.id = f"csv_replay_{side}_{seq}"
    msg.goal.width = width
    msg.goal.speed = speed
    return msg


def find_controller(controllers, name):
    for controller in controllers:
        if controller.name == name:
            return controller
    return None


def check_controller(rospy, ListControllers, service, name, expected_type):
    rospy.wait_for_service(service, timeout=5.0)
    controllers = ListControllers._request_class
    response = rospy.ServiceProxy(service, ListControllers)(controllers())
    controller = find_controller(response.controller, name)
    if controller is None:
        raise RuntimeError(f"{service}: controller '{name}' is not loaded")
    if controller.state != "running":
        raise RuntimeError(f"{service}: controller '{name}' is {controller.state}, not running")
    actual_type = getattr(controller, "type", "")
    if actual_type and actual_type != expected_type:
        raise RuntimeError(
            f"{service}: controller '{name}' type is '{actual_type}', expected '{expected_type}'"
        )


def check_controllers(rospy, ListControllers, args):
    if args.skip_controller_check:
        return
    if args.control_mode == "joint":
        expected = "fluxvla_franka_controllers/RuckigJointImpedanceController"
        left = args.left_joint_controller
        right = args.right_joint_controller
    else:
        expected = "serl_franka_controllers/CartesianImpedanceController"
        left = args.left_eepose_controller
        right = args.right_eepose_controller
    check_controller(rospy, ListControllers, args.left_controller_manager, left, expected)
    check_controller(rospy, ListControllers, args.right_controller_manager, right, expected)


def print_summary(rows, args):
    duration = len(rows) / max(args.replay_hz, 1e-6)
    print("========== replay_joint_impedance_csv ==========")
    print(f"csv: {Path(args.csv).expanduser()}")
    print(f"control_mode: {args.control_mode}")
    print(f"frames: {len(rows)}, replay_hz: {args.replay_hz:.2f}, duration: {duration:.3f}s")
    if args.control_mode == "joint":
        print(f"left topic: {args.left_topic}")
        print(f"right topic: {args.right_topic}")
    else:
        print(f"left pose topic: {args.left_pose_topic}")
        print(f"right pose topic: {args.right_pose_topic}")
    print(f"gripper: {'disabled' if args.no_gripper else 'enabled when CSV columns exist'}")
    if not args.execute:
        print("dry-run: add --execute to publish ROS messages")


def main():
    args = parse_args()
    rows = read_csv(args.csv, args.control_mode)
    print_summary(rows, args)
    if not args.execute:
        return

    import rospy
    from controller_manager_msgs.srv import ListControllers
    from franka_gripper.msg import MoveActionGoal
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState

    rospy.init_node("replay_joint_impedance_csv", anonymous=True)
    check_controllers(rospy, ListControllers, args)

    left_pub = rospy.Publisher(args.left_topic, JointState, queue_size=10, tcp_nodelay=True)
    right_pub = rospy.Publisher(args.right_topic, JointState, queue_size=10, tcp_nodelay=True)
    left_pose_pub = rospy.Publisher(args.left_pose_topic, PoseStamped, queue_size=10)
    right_pose_pub = rospy.Publisher(args.right_pose_topic, PoseStamped, queue_size=10)
    left_gripper_pub = rospy.Publisher(args.left_gripper_topic, MoveActionGoal, queue_size=10)
    right_gripper_pub = rospy.Publisher(args.right_gripper_topic, MoveActionGoal, queue_size=10)

    rate = rospy.Rate(args.replay_hz)
    for seq, row in enumerate(rows):
        if rospy.is_shutdown():
            break
        if args.control_mode == "joint":
            left_pub.publish(make_joint_state(rospy, JointState, row["left"]))
            right_pub.publish(make_joint_state(rospy, JointState, row["right"]))
        else:
            left_pose_pub.publish(
                make_pose_stamped(rospy, PoseStamped, row["left"], args.left_frame_id)
            )
            right_pose_pub.publish(
                make_pose_stamped(rospy, PoseStamped, row["right"], args.right_frame_id)
            )
        if not args.no_gripper:
            if row["left_gripper"] is not None:
                left_gripper_pub.publish(
                    make_gripper_goal(
                        rospy, MoveActionGoal, row["left_gripper"], args.gripper_speed, "left", seq
                    )
                )
            if row["right_gripper"] is not None:
                right_gripper_pub.publish(
                    make_gripper_goal(
                        rospy, MoveActionGoal, row["right_gripper"], args.gripper_speed, "right", seq
                    )
                )
        rate.sleep()


if __name__ == "__main__":
    main()
