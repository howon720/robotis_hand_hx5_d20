#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math


class HandTrajTeleop(Node):
    def __init__(self):
        super().__init__('hand_traj_teleop')

        # joint 순서
        self.joint_names = [
            "finger_l_joint1", "finger_l_joint2", "finger_l_joint3", "finger_l_joint4",
            "finger_l_joint5", "finger_l_joint6", "finger_l_joint7", "finger_l_joint8",
            "finger_l_joint9", "finger_l_joint10", "finger_l_joint11", "finger_l_joint12",
            "finger_l_joint13", "finger_l_joint14", "finger_l_joint15", "finger_l_joint16",
            "finger_l_joint17", "finger_l_joint18", "finger_l_joint19", "finger_l_joint20",
        ]

        # 퍼블리셔
        self.cmd_pub = self.create_publisher(
            JointTrajectory,
            '/left_hand_controller/joint_trajectory',
            10
        )
        self.step_interval = 0.2

        # trajectory step 목록 생성
        self.traj_steps = self.build_traj_steps()

        # 현재 step index
        self.current_step_idx = 0

        # 0.2초 주기 timer
        self.timer = self.create_timer(self.step_interval, self.timer_callback)

        self.get_logger().info('Trajectory repeat mode started.')

    def deg2rad(self, deg):
        return deg * math.pi / 180.0

    def base_positions_dict(self, default_deg=0.0):
        """모든 joint를 default 값으로 초기화"""
        base = {}
        for name in self.joint_names:
            base[name] = self.deg2rad(default_deg)
        return base

    def finger_pose(self, start_joint_num, values_deg):
        pose = {}
        for i, val in enumerate(values_deg):
            joint_num = start_joint_num + i
            joint_name = f'finger_l_joint{joint_num}'
            pose[joint_name] = self.deg2rad(val)
        return pose

    def build_traj_steps(self):
        """반복할 trajectory step 구성"""
        # =========================
        home_pose = self.base_positions_dict(0.0)
        home_pose.update(self.finger_pose(1,  [-40.6, 59.2, 29.5, -48.3]))   # Finger 1
        home_pose.update(self.finger_pose(5,  [24.3, 51.9, 8.4, 41.2]))      # Finger 2
        home_pose.update(self.finger_pose(9,  [2.8, 33.8, 17.1, 39.0]))      # Finger 3
        home_pose.update(self.finger_pose(13, [-12.7, 44.0, 8.3, 38.7]))     # Finger 4
        home_pose.update(self.finger_pose(17, [-20.4, 35.7, 26.1, 38.2]))    # Finger 5

        release_pose = self.base_positions_dict(0.0)
        release_pose.update(self.finger_pose(1,  [-6.1, -2.1, -41.4, -41.0]))   # Finger 1

        grisp = self.base_positions_dict(90.0)
        grisp.update(self.finger_pose(1,  [-6.1, -2.1, -41.4, -41.0]))   # Finger 1
        grisp["finger_l_joint5"] = self.deg2rad(0.0)
        grisp["finger_l_joint9"] = self.deg2rad(0.0)
        grisp["finger_l_joint13"] = self.deg2rad(0.0)
        grisp["finger_l_joint17"] = self.deg2rad(0.0)

        # =========================
        step1 = home_pose.copy()
        step1.update(self.finger_pose(1,  [-33.4, 59.2, -22.9, -21.4]))      # Finger 1
        step1.update(self.finger_pose(5,  [24.2, 60.4, 17.2, 40.7]))         # Finger 2

        step2 = home_pose.copy()
        step2.update(self.finger_pose(1,  [-18.2, 107.9, -8.3, -91.6]))      # Finger 1
        step2.update(self.finger_pose(9,  [2.8, 48.0, 83.7, 28.2]))          # Finger 3

        step3 = home_pose.copy()
        step3.update(self.finger_pose(1,  [-25.9, 116.3, -8.3, -91.6]))      # Finger 1
        step3.update(self.finger_pose(13, [-14.5, 54.5, 71.3, 29.4]))        # Finger 4

        step4 = home_pose.copy()
        step4.update(self.finger_pose(1,  [-51.4, 131.0, -30.0, -60.2]))     # Finger 1
        step4.update(self.finger_pose(17, [-17.8, 62.6, 71.4, 54.0]))        # Finger 5

        # =========================
        step5 = release_pose.copy()
        step5.update(self.finger_pose(1,  [0.0, -2.0, -23.8, -59.0]))
        step5.update(self.finger_pose(5,  [15.0, 0.0, 0.0, 0.0]))
        step5.update(self.finger_pose(9,  [10.0, 0.0, 0.0, 0.0]))
        step5.update(self.finger_pose(13, [-10.0, 0.0, 0.0, 0.0]))
        step5.update(self.finger_pose(17, [-22.0, 0.0, 0.0, 0.0]))


        # 반복 순서
        # if(self.step_interval < 0.3):
        return [home_pose, step1, step2, step3, step4, release_pose, grisp, step5, release_pose, step5]
        # else:
        #     return [release_pose, grisp, step5, release_pose, step5]

    def reset_timer(self, new_interval):
        self.timer.cancel()
        self.timer = self.create_timer(new_interval, self.timer_callback)

    def timer_callback(self):
        """0.2초마다 현재 step publish"""
        target_pose = self.traj_steps[self.current_step_idx]

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [target_pose[joint] for joint in self.joint_names]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000  # 0.2 sec  #0.2

        msg.points.append(point)

        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'Published step {self.current_step_idx}: '
            + ', '.join([f'{j}={round(math.degrees(target_pose[j]), 1)}deg' for j in self.joint_names[:4]])
        )

        # # 다음 step으로 이동, 끝이면 다시 처음
        # self.current_step_idx = (self.current_step_idx + 1) % len(self.traj_steps)

        self.current_step_idx += 1

        # 한 바퀴 end시
        if self.current_step_idx >= len(self.traj_steps):
            self.current_step_idx = 0

            # 토글
            if abs(self.step_interval - 0.2) < 1e-6:
                self.step_interval = 0.5
            else:
                self.step_interval = 0.2

            self.get_logger().info(
                f'Cycle finished. Next cycle interval = {self.step_interval:.1f}s'
            )
            self.reset_timer(self.step_interval)


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
