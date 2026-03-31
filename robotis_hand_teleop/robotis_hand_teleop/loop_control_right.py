#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
import math


class HandTrajTeleop(Node):
    def __init__(self):
        super().__init__('hand_traj_teleop')

        self.joint_names = [
            "finger_r_joint1", "finger_r_joint2", "finger_r_joint3", "finger_r_joint4",
            "finger_r_joint5", "finger_r_joint6", "finger_r_joint7", "finger_r_joint8",
            "finger_r_joint9", "finger_r_joint10", "finger_r_joint11", "finger_r_joint12",
            "finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16",
            "finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20",
        ]

        self.vr_stream_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cmd_pub = self.create_publisher(
            JointTrajectory,
            '/right_hand_controller/joint_trajectory',
            self.vr_stream_qos
        )

        # 사이클 사이 pause
        self.cycle_end = 0.8

        # 고정 타이머
        self.check_period = 0.1

        self.fast_steps, self.slow_steps, self.fast_durations, self.slow_durations = self.build_traj_steps()

        self.current_cycle_is_fast = False   # start : slow
        self.current_pose = self.slow_steps[0].copy()

        self.next_cycle_time = self.get_clock().now()

        self.timer = self.create_timer(self.check_period, self.timer_callback)

        self.get_logger().info('Trajectory repeat mode started.')

    def deg2rad(self, deg):
        return deg * math.pi / 180.0

    def base_positions_dict(self, default_deg=0.0):
        base = {}
        for name in self.joint_names:
            base[name] = self.deg2rad(default_deg)
        return base

    def finger_pose(self, start_joint_num, values_deg):
        pose = {}
        for i, val in enumerate(values_deg):
            joint_num = start_joint_num + i
            joint_name = f'finger_r_joint{joint_num}'
            pose[joint_name] = self.deg2rad(val)
        return pose

    def build_traj_steps(self):
        home_pose = self.base_positions_dict(0.0)
        home_pose.update(self.finger_pose(1,  [40.6, -59.2, -29.5, 48.3]))
        home_pose.update(self.finger_pose(5,  [-24.3, 51.9, 8.4, 41.2]))
        home_pose.update(self.finger_pose(9,  [-2.8, 33.8, 17.1, 39.0]))
        home_pose.update(self.finger_pose(13, [12.7, 44.0, 8.3, 38.7]))
        home_pose.update(self.finger_pose(17, [20.4, 35.7, 26.1, 38.2]))

        home_pose_setup = self.base_positions_dict(0.0)
        home_pose_setup["finger_r_joint5"] = self.deg2rad(-24.3)
        home_pose_setup["finger_r_joint9"] = self.deg2rad(-2.8)
        home_pose_setup["finger_r_joint13"] = self.deg2rad(12.7)
        home_pose_setup["finger_r_joint17"] = self.deg2rad(20.4)

        release_pose = self.base_positions_dict(0.0)
        release_pose.update(self.finger_pose(1,  [6.1, 2.1, 41.4, 41.0]))

        grisp = self.base_positions_dict(90.0)
        grisp.update(self.finger_pose(1,  [6.1, 2.1, 41.4, 41.0]))
        grisp["finger_r_joint5"] = self.deg2rad(0.0)
        grisp["finger_r_joint9"] = self.deg2rad(0.0)
        grisp["finger_r_joint13"] = self.deg2rad(0.0)
        grisp["finger_r_joint17"] = self.deg2rad(0.0)

        release_pose1 = grisp.copy()
        release_pose1["finger_r_joint1"] = 0.821
        release_pose1["finger_r_joint2"] = 0.0
        release_pose1["finger_r_joint3"] = 0.6
        release_pose1["finger_r_joint4"] = 0.474

        step1 = home_pose.copy()
        step1.update(self.finger_pose(1,  [33.4, -59.2, 25.9, 18.4]))
        step1.update(self.finger_pose(5,  [-24.2, 68.4, 17.2, 40.7]))

        step2 = home_pose.copy()
        step2.update(self.finger_pose(1,  [18.2, -107.9, 11.3, 88.6]))
        step2.update(self.finger_pose(9,  [-2.8, 56.0, 83.7, 28.2]))

        step3 = home_pose.copy()
        step3.update(self.finger_pose(1,  [25.9, -116.3, 11.3, 88.6]))
        step3.update(self.finger_pose(13, [14.5, 60.5, 71.3, 29.4]))

        step4 = home_pose.copy()
        step4.update(self.finger_pose(1,  [51.4, -131.0, 33.0, 57.2]))
        step4.update(self.finger_pose(17, [17.8, 68.6, 71.4, 54.0]))

        # fast / slow step 리스트
        fast_steps = [home_pose_setup, home_pose, step1, step2, step3, step4, step3, step2, step1, home_pose, release_pose1]
        slow_steps = [release_pose, home_pose_setup, home_pose, step1, step2, step3, step4, step3, step2, step1,
                      release_pose, grisp, grisp, release_pose]

        # 도착 시간 간격
        fast_durations = [0.2] * len(fast_steps)
        slow_durations = [0.5] * len(slow_steps)

        return fast_steps, slow_steps, fast_durations, slow_durations

    def interpolate_pose(self, start_pose, goal_pose, alpha):
        interp_pose = {}
        for joint in self.joint_names:
            s = start_pose[joint]
            g = goal_pose[joint]
            interp_pose[joint] = s + alpha * (g - s)
        return interp_pose

    def make_cycle_trajectory(self, start_pose, steps, durations):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        prev_pose = start_pose.copy()
        cumulative_t = 0.0

        for target_pose, duration in zip(steps, durations):
            # duration별 보간 포인트 수
            if duration <= 0.2:
                num_points = 2
            elif duration <= 0.5:
                num_points = 4
            else:
                num_points = 6

            for i in range(1, num_points + 1):
                alpha = i / num_points
                interp_pose = self.interpolate_pose(prev_pose, target_pose, alpha)

                point = JointTrajectoryPoint()
                point.positions = [interp_pose[j] for j in self.joint_names]

                point_t = cumulative_t + alpha * duration
                sec = int(point_t)
                nanosec = int((point_t - sec) * 1e9)
                point.time_from_start.sec = sec
                point.time_from_start.nanosec = nanosec

                msg.points.append(point)

            cumulative_t += duration
            prev_pose = target_pose.copy()

        return msg, cumulative_t, prev_pose

    def timer_callback(self):
        now = self.get_clock().now()

        # cycle end 전 행동 X
        if now < self.next_cycle_time:
            return

        if self.current_cycle_is_fast:
            steps = self.fast_steps
            durations = self.fast_durations
            cycle_name = "fast"
        else:
            steps = self.slow_steps
            durations = self.slow_durations
            cycle_name = "slow"

        msg, cycle_duration, final_pose = self.make_cycle_trajectory(
            self.current_pose,
            steps,
            durations
        )
        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'Published {cycle_name} cycle: steps={len(steps)}, duration={cycle_duration:.2f}s, points={len(msg.points)}'
        )

        self.current_pose = final_pose
        self.next_cycle_time = now + Duration(seconds=cycle_duration + self.cycle_end)

        # 다음 cycle 토글
        self.current_cycle_is_fast = not self.current_cycle_is_fast


def main(args=None):
    rclpy.init(args=args)
    node = HandTrajTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down by Ctrl+C')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()