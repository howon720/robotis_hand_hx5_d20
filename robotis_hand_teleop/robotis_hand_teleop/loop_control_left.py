#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

## DYNAMICXEL 각도 보고 SET
class HandTrajTeleop(Node):
    def __init__(self):
        super().__init__('hand_traj_teleop')

        self.joint_names = [
            "finger_l_joint1", "finger_l_joint2", "finger_l_joint3", "finger_l_joint4",
            "finger_l_joint5", "finger_l_joint6", "finger_l_joint7", "finger_l_joint8",
            "finger_l_joint9", "finger_l_joint10", "finger_l_joint11", "finger_l_joint12",
            "finger_l_joint13", "finger_l_joint14", "finger_l_joint15", "finger_l_joint16",
            "finger_l_joint17", "finger_l_joint18", "finger_l_joint19", "finger_l_joint20",
        ]

        self.cmd_pub = self.create_publisher(
            JointTrajectory,
            '/left_hand_controller/joint_trajectory',
            10
        )

        self.cycle_end = 3.0
        self.waiting = False

        self.step_interval = 0.8
        self.fast_steps, self.slow_steps = self.build_traj_steps()
        self.traj_steps = self.slow_steps
        self.current_step_idx = 0
        # 시작자세 첫 pose
        self.current_pose = self.traj_steps[0].copy()

        self.timer = self.create_timer(self.step_interval, self.timer_callback)

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
            joint_name = f'finger_l_joint{joint_num}'
            pose[joint_name] = self.deg2rad(val)
        return pose

    def build_traj_steps(self):
        home_pose = self.base_positions_dict(0.0)
        home_pose.update(self.finger_pose(1,  [-40.6, 59.2, 29.5, -48.3]))
        home_pose.update(self.finger_pose(5,  [24.3, 51.9, 8.4, 41.2]))
        home_pose.update(self.finger_pose(9,  [2.8, 33.8, 17.1, 39.0]))
        home_pose.update(self.finger_pose(13, [-12.7, 44.0, 8.3, 38.7]))
        home_pose.update(self.finger_pose(17, [-20.4, 35.7, 26.1, 38.2]))

        release_pose = self.base_positions_dict(0.0)
        release_pose.update(self.finger_pose(1,  [-6.1, -2.1, -41.4, -41.0]))   # Finger 1

        grisp = self.base_positions_dict(90.0)
        grisp.update(self.finger_pose(1,  [-6.1, -2.1, -41.4, -41.0]))   # Finger 1
        grisp["finger_l_joint5"] = self.deg2rad(0.0)
        grisp["finger_l_joint9"] = self.deg2rad(0.0)
        grisp["finger_l_joint13"] = self.deg2rad(0.0)
        grisp["finger_l_joint17"] = self.deg2rad(0.0)

        # ==========================
        step1 = home_pose.copy()
        step1.update(self.finger_pose(1,  [-33.4, 59.2, -22.9, -21.4]))
        step1.update(self.finger_pose(5,  [24.2, 60.4, 17.2, 40.7]))

        step2 = home_pose.copy()
        step2.update(self.finger_pose(1,  [-18.2, 107.9, -8.3, -91.6]))
        step2.update(self.finger_pose(9,  [2.8, 48.0, 83.7, 28.2]))

        step3 = home_pose.copy()
        step3.update(self.finger_pose(1,  [-25.9, 116.3, -8.3, -91.6]))
        step3.update(self.finger_pose(13, [-14.5, 54.5, 71.3, 29.4]))

        step4 = home_pose.copy()
        step4.update(self.finger_pose(1,  [-51.4, 131.0, -30.0, -60.2]))
        step4.update(self.finger_pose(17, [-17.8, 62.6, 71.4, 54.0]))
        # ==========================
        step5 = release_pose.copy()
        step5.update(self.finger_pose(1,  [0.0, -1.0, -38.0, -50.0]))
        step5.update(self.finger_pose(5,  [15.0, 0.0, 0.0, 0.0]))
        step5.update(self.finger_pose(9,  [5.0, 0.0, 0.0, 0.0]))
        step5.update(self.finger_pose(13, [-5.0, 0.0, 0.0, 0.0]))
        step5.update(self.finger_pose(17, [-15.0, 0.0, 0.0, 0.0]))
        
        step6 = release_pose.copy()
        step6.update(self.finger_pose(1,  [0.0, -2.0, -23.8, -59.0]))
        step6.update(self.finger_pose(5,  [24.0, 0.0, 0.0, 0.0]))
        step6.update(self.finger_pose(9,  [8.0, 0.0, 0.0, 0.0]))
        step6.update(self.finger_pose(13, [-8.0, 0.0, 0.0, 0.0]))
        step6.update(self.finger_pose(17, [-24.0, 0.0, 0.0, 0.0]))
        # return [home_pose, step1, step2, step3, step4, release_pose, grisp, step5, release_pose, step5]

        fast_steps = [home_pose, step1, step2, step3, step4, step3, step2, step1, home_pose]
        slow_steps = [home_pose, step1, step2, step3, step4, step3, step2, step1,
                    release_pose, grisp, release_pose, step5, release_pose, step6, release_pose, grisp]

        return fast_steps, slow_steps

    def interpolate_pose(self, start_pose, goal_pose, alpha):
        interp_pose = {}
        for joint in self.joint_names:
            s = start_pose[joint]
            g = goal_pose[joint]
            interp_pose[joint] = s + alpha * (g - s)
        return interp_pose

    def make_interpolated_trajectory(self, start_pose, goal_pose, duration, num_points):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        for i in range(1, num_points + 1):
            alpha = i / num_points
            interp_pose = self.interpolate_pose(start_pose, goal_pose, alpha)

            point = JointTrajectoryPoint()
            point.positions = [interp_pose[j] for j in self.joint_names]

            t = alpha * duration
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            point.time_from_start.sec = sec
            point.time_from_start.nanosec = nanosec

            msg.points.append(point)

        return msg

    def reset_timer(self, new_interval):
        self.timer.cancel()
        self.timer = self.create_timer(new_interval, self.timer_callback)

    def timer_callback(self):
        if self.waiting:
            self.waiting = False
            self.reset_timer(self.step_interval)

        target_pose = self.traj_steps[self.current_step_idx]

        # interval에 따라 보간 포인트 수 결정
        if self.step_interval <= 0.2:
            num_points = 5
        else:
            num_points = 12

        msg = self.make_interpolated_trajectory(
            self.current_pose,
            target_pose,
            self.step_interval,
            num_points
        )
        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'step={self.current_step_idx}, interval={self.step_interval:.1f}s, points={num_points}'
        )

        # 현재 pose를 목표 pose로 갱신
        self.current_pose = target_pose.copy()

        self.current_step_idx += 1

        # 한 사이클 종료시 토글
        if self.current_step_idx >= len(self.traj_steps):
            self.current_step_idx = 0
            

            if abs(self.step_interval - 0.2) < 1e-6:
                self.step_interval = 0.6
                self.traj_steps = self.slow_steps
            else:
                self.step_interval = 0.2
                self.traj_steps = self.fast_steps

            self.get_logger().info(
                f'Cycle finished. Next cycle interval = {self.step_interval:.1f}s, '
                f'steps = {len(self.traj_steps)}'
            )
            self.reset_timer(self.cycle_end)
            self.waiting = True
            return


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