#!/usr/bin/env python3
"""Replay qpos CSV to Ruckig joint impedance controllers at a fixed rate."""

import argparse
import csv
from pathlib import Path


JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
LEFT_Q = [f"left_q{i}" for i in range(1, 8)]
RIGHT_Q = [f"right_q{i}" for i in range(1, 8)]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Replay CSV qpos to RuckigJointImpedanceController."
    )
    parser.add_argument("--csv", required=True, help="CSV with left/right qpos columns")
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


def read_csv(path):
    path = Path(path).expanduser()
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        require_columns(fieldnames, LEFT_Q + RIGHT_Q)
        rows = []
        for row in reader:
            rows.append(
                {
                    "left": [float(row[name]) for name in LEFT_Q],
                    "right": [float(row[name]) for name in RIGHT_Q],
                    "left_gripper": read_float(row, "left_gripper"),
                    "right_gripper": read_float(row, "right_gripper"),
                }
            )
    if not rows:
        raise RuntimeError(f"CSV has no data rows: {path}")
    return rows


def make_joint_state(rospy, JointState, q):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = JOINT_NAMES
    msg.position = q
    return msg


def make_gripper_goal(rospy, MoveActionGoal, width, speed, side, seq):
    msg = MoveActionGoal()
    msg.header.stamp = rospy.Time.now()
    msg.goal_id.stamp = msg.header.stamp
    msg.goal_id.id = f"csv_replay_{side}_{seq}"
    msg.goal.width = width
    msg.goal.speed = speed
    return msg


def print_summary(rows, args):
    duration = len(rows) / max(args.replay_hz, 1e-6)
    print("========== replay_joint_impedance_csv ==========")
    print(f"csv: {Path(args.csv).expanduser()}")
    print(f"frames: {len(rows)}, replay_hz: {args.replay_hz:.2f}, duration: {duration:.3f}s")
    print(f"left topic: {args.left_topic}")
    print(f"right topic: {args.right_topic}")
    print(f"gripper: {'disabled' if args.no_gripper else 'enabled when CSV columns exist'}")
    if not args.execute:
        print("dry-run: add --execute to publish ROS messages")


def main():
    args = parse_args()
    rows = read_csv(args.csv)
    print_summary(rows, args)
    if not args.execute:
        return

    import rospy
    from franka_gripper.msg import MoveActionGoal
    from sensor_msgs.msg import JointState

    rospy.init_node("replay_joint_impedance_csv", anonymous=True)
    left_pub = rospy.Publisher(args.left_topic, JointState, queue_size=10, tcp_nodelay=True)
    right_pub = rospy.Publisher(args.right_topic, JointState, queue_size=10, tcp_nodelay=True)
    left_gripper_pub = rospy.Publisher(args.left_gripper_topic, MoveActionGoal, queue_size=10)
    right_gripper_pub = rospy.Publisher(args.right_gripper_topic, MoveActionGoal, queue_size=10)

    rate = rospy.Rate(args.replay_hz)
    for seq, row in enumerate(rows):
        if rospy.is_shutdown():
            break
        left_pub.publish(make_joint_state(rospy, JointState, row["left"]))
        right_pub.publish(make_joint_state(rospy, JointState, row["right"]))
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
