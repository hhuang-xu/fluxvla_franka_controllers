# Copyright 2026 Limx Dynamics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import threading
from collections import defaultdict
from datetime import datetime


DEFAULT_LEFT_COMMAND_TOPIC = (
    '/left_arm/ruckig_joint_impedance_controller/target_joint_state')
DEFAULT_RIGHT_COMMAND_TOPIC = (
    '/right_arm/ruckig_joint_impedance_controller/target_joint_state')
DEFAULT_LEFT_STATE_TOPIC = '/left_arm/joint_states'
DEFAULT_RIGHT_STATE_TOPIC = '/right_arm/joint_states'


def parse_args():
    parser = argparse.ArgumentParser(
        description='Record and plot commanded vs actual JointState topics.')
    parser.add_argument(
        '--command-topic',
        action='append',
        dest='command_topics',
        default=None,
        help='Commanded JointState topic. Repeat for multiple arms.')
    parser.add_argument(
        '--state-topic',
        action='append',
        dest='state_topics',
        default=None,
        help='Actual/state JointState topic. Repeat for multiple arms.')
    parser.add_argument(
        '--name',
        action='append',
        dest='names',
        default=None,
        help='Plot column name for each command/state topic pair.')
    parser.add_argument(
        '--duration',
        type=float,
        default=0.0,
        help='Recording duration in seconds. Use 0 to record until Ctrl+C.')
    parser.add_argument(
        '--output',
        default=None,
        help='Output image path. Defaults to a timestamped PNG.')
    parser.add_argument(
        '--show',
        action='store_true',
        help='Show the plot window after recording.')
    return parser.parse_args()


def build_topic_pairs(args):
    if args.command_topics is None and args.state_topics is None:
        return [
            {
                'name': 'left',
                'command_topic': DEFAULT_LEFT_COMMAND_TOPIC,
                'state_topic': DEFAULT_LEFT_STATE_TOPIC,
            },
            {
                'name': 'right',
                'command_topic': DEFAULT_RIGHT_COMMAND_TOPIC,
                'state_topic': DEFAULT_RIGHT_STATE_TOPIC,
            },
        ]

    if not args.command_topics or not args.state_topics:
        raise ValueError(
            '--command-topic and --state-topic must be provided together')
    if len(args.command_topics) != len(args.state_topics):
        raise ValueError(
            '--command-topic and --state-topic must have the same count')
    if args.names is not None and len(args.names) != len(args.command_topics):
        raise ValueError('--name count must match --command-topic count')

    names = args.names or [
        f'joint_state_{idx + 1}' for idx in range(len(args.command_topics))
    ]
    return [
        {
            'name': names[idx],
            'command_topic': command_topic,
            'state_topic': args.state_topics[idx],
        }
        for idx, command_topic in enumerate(args.command_topics)
    ]


class JointTrajectoryRecorder:

    def __init__(self, start_time):
        self.data = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
        self.lock = threading.Lock()
        self.start_time = float(start_time)

    def callback(self, group_name, source, msg):
        stamp = self._msg_time(msg)
        values = self._joint_positions(msg)
        if values is None:
            return

        with self.lock:
            rel_time = max(0.0, stamp - self.start_time)
            for joint_name, value in values.items():
                self.data[group_name][source][joint_name].append(
                    (rel_time, float(value)))

    @staticmethod
    def _msg_time(msg):
        import rospy

        stamp = msg.header.stamp
        if stamp is None or stamp.to_sec() == 0.0:
            stamp = rospy.Time.now()
        return stamp.to_sec()

    @staticmethod
    def _joint_positions(msg):
        if not msg.position:
            return None

        positions = {}
        if msg.name:
            for idx, (joint_name, value) in enumerate(
                    zip(msg.name, msg.position)):
                positions[joint_name] = value
                positions[f'joint_{idx + 1}'] = value
            return positions

        for idx, value in enumerate(msg.position):
            positions[f'joint_{idx + 1}'] = value
        return positions

    def snapshot(self):
        with self.lock:
            return {
                side: {
                    source: {
                        joint: list(samples)
                        for joint, samples in joints.items()
                    }
                    for source, joints in sources.items()
                }
                for side, sources in self.data.items()
            }


def subscribe_topics(topic_pairs, recorder):
    import rospy
    from sensor_msgs.msg import JointState

    subscriptions = []
    for pair in topic_pairs:
        subscriptions.extend([
            (pair['name'], 'command', pair['command_topic']),
            (pair['name'], 'actual', pair['state_topic']),
        ])

    for group_name, source, topic in subscriptions:
        rospy.Subscriber(
            topic,
            JointState,
            lambda msg, group_name=group_name, source=source: recorder.callback(
                group_name, source, msg),
            queue_size=1000,
            tcp_nodelay=True)
        rospy.loginfo('Subscribed %s %s trajectory: %s', group_name, source,
                      topic)


def wait_for_recording(duration):
    import rospy

    if duration > 0.0:
        rospy.loginfo('Recording %.3f seconds...', duration)
        rospy.sleep(duration)
        return

    rospy.loginfo('Recording until Ctrl+C...')
    rospy.spin()


def split_samples(samples):
    if not samples:
        return [], []
    times, values = zip(*samples)
    return list(times), list(values)


def is_positional_joint_name(joint_name):
    suffix = joint_name.removeprefix('joint_')
    return joint_name.startswith('joint_') and suffix.isdigit()


def ordered_joint_names(joints):
    return list(joints.keys())


def select_plot_joints(group_data):
    command_joints = ordered_joint_names(group_data.get('command', {}))
    actual_joints = ordered_joint_names(group_data.get('actual', {}))
    common_joints = [
        joint_name for joint_name in command_joints
        if joint_name in set(actual_joints)
    ]
    named_common = [
        joint_name for joint_name in common_joints
        if not is_positional_joint_name(joint_name)
    ]
    if named_common:
        return named_common
    if common_joints:
        return common_joints
    if command_joints:
        return command_joints
    return actual_joints


def plot_trajectories(data, topic_pairs, output, show):
    import matplotlib.pyplot as plt
    import rospy

    group_names = [pair['name'] for pair in topic_pairs]
    joints_by_group = {
        group_name: select_plot_joints(data.get(group_name, {}))
        for group_name in group_names
    }
    num_rows = max((len(joints) for joints in joints_by_group.values()),
                   default=1)
    num_cols = max(len(group_names), 1)

    fig, axes = plt.subplots(
        num_rows,
        num_cols,
        figsize=(7 * num_cols, 2.6 * num_rows),
        sharex='col',
        squeeze=False)
    fig.suptitle('JointState Trajectory: Command vs Actual')

    for col, group_name in enumerate(group_names):
        joint_names = joints_by_group[group_name]
        if not joint_names:
            rospy.logwarn('No JointState samples recorded for %s', group_name)
        for row in range(num_rows):
            ax = axes[row][col]
            if row >= len(joint_names):
                ax.axis('off')
                continue

            joint_name = joint_names[row]
            group_data = data.get(group_name, {})
            command = group_data.get('command', {}).get(joint_name, [])
            actual = group_data.get('actual', {}).get(joint_name, [])
            command_t, command_q = split_samples(command)
            actual_t, actual_q = split_samples(actual)

            if command_t:
                ax.plot(command_t, command_q, label='command', linewidth=1.2)
            if actual_t:
                ax.plot(actual_t, actual_q, label='actual', linewidth=1.2)
            if not command_t or not actual_t:
                rospy.logwarn(
                    'Missing %s %s data: command=%d samples, actual=%d samples',
                    group_name,
                    joint_name,
                    len(command),
                    len(actual))

            ax.set_title(f'{group_name} {joint_name}')
            ax.set_ylabel('position [rad]')
            ax.set_xlim(left=0.0)
            ax.grid(True)
            if row == num_rows - 1:
                ax.set_xlabel('time [s]')
            if row == 0:
                ax.legend(loc='best')

    fig.tight_layout(rect=(0, 0, 1, 0.98))
    fig.savefig(output, dpi=150)
    rospy.loginfo('Saved trajectory plot to %s', output)

    if show:
        plt.show()
    else:
        plt.close(fig)


def default_output_path():
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return f'jointstate_trajectory_{timestamp}.png'


def main():
    args = parse_args()
    topic_pairs = build_topic_pairs(args)
    output = args.output or default_output_path()

    import rospy

    rospy.init_node('plot_jointstate_trajectory', anonymous=True)
    recorder = JointTrajectoryRecorder(rospy.Time.now().to_sec())
    subscribe_topics(topic_pairs, recorder)

    try:
        wait_for_recording(args.duration)
    except KeyboardInterrupt:
        pass

    plot_trajectories(recorder.snapshot(), topic_pairs, output, args.show)


if __name__ == '__main__':
    main()


# Usage examples:
#
# Single arm:
# python scripts/plot_jointstate_trajectory.py \
#   --command-topic /left_arm/ruckig_joint_impedance_controller/target_joint_state \
#   --state-topic /left_arm/joint_states \
#   --name left \
#   --duration 30 \
#   --output left_joint_trajectory.png
#
# Dual arms:
# python scripts/plot_jointstate_trajectory.py \
#   --command-topic /left_arm/ruckig_joint_impedance_controller/target_joint_state \
#   --state-topic /left_arm/joint_states \
#   --name left \
#   --command-topic /right_arm/ruckig_joint_impedance_controller/target_joint_state \
#   --state-topic /right_arm/joint_states \
#   --name right \
#   --duration 30 \
#   --output dual_joint_trajectory.png
