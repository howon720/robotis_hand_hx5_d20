#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HandStepControl(Node):
    def __init__(self):
        super().__init__('hand_step_control')

        self.joint_names = [
            "finger_r_joint1", "finger_r_joint2", "finger_r_joint3", "finger_r_joint4",
            "finger_r_joint5", "finger_r_joint6", "finger_r_joint7", "finger_r_joint8",
            "finger_r_joint9", "finger_r_joint10", "finger_r_joint11", "finger_r_joint12",
            "finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16",
            "finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20",
        ]

        self.cmd_pub = self.create_publisher(
            JointTrajectory,
            '/right_hand_controller/joint_trajectory',
            10
        )

        # 이동시간 / 대기시간
        self.move_time = 0.2
        self.pause_time = 0.3

        # publish period = move + pause
        self.timer_period = self.move_time + self.pause_time

        self.traj_steps = self.build_traj_steps()
        self.current_step_idx = 0

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            f'Started step control. move_time={self.move_time:.1f}s, '
            f'pause_time={self.pause_time:.1f}s, total_period={self.timer_period:.1f}s'
        )  # 주먹 직선피기 펼치기 다시직선 다시 펼치고 이제시작 

    def base_positions_dict(self, default_val=0.0):
        return {name: default_val for name in self.joint_names}

    def finger_pose(self, start_joint_num, values_rad):
        pose = {}
        for i, val in enumerate(values_rad):
            joint_num = start_joint_num + i
            joint_name = f'finger_r_joint{joint_num}'
            pose[joint_name] = val
        return pose

    def build_traj_steps(self):

        base_set = self.base_positions_dict(0.0)
        base_set.update(self.finger_pose(1,  [-0.11, -0.0, 1.0, 0.5]))
        base_set["finger_r_joint5"] = -0.1
        base_set["finger_r_joint9"] = 0.0
        base_set["finger_r_joint13"] = 0.1
        base_set["finger_r_joint17"] = 0.2

        common_pose1 = [-0.100, 0.962, 1.095, 0.823]
        common_pose2 = [0.0, 0.962, 1.095, 0.823]
        common_pose3 = [0.100, 0.962, 1.095, 0.823]

        release_pose = self.base_positions_dict(0.0)
        release_pose.update(self.finger_pose(1,  [0.106, 0.036, 0.723, 0.723]))

        grisp = self.base_positions_dict(1.57)
        grisp.update(self.finger_pose(1,  [0.106, 0.036, 0.723, 0.723]))
        grisp["finger_r_joint5"] = 0.0
        grisp["finger_r_joint9"] = 0.0
        grisp["finger_r_joint13"] = 0.0
        grisp["finger_r_joint17"] = 0.0

        traj_steps = []

        ## step 시작
        # traj_steps.append(grisp)

        traj_steps.append(release_pose)

        step5 = release_pose.copy()
        step5.update(self.finger_pose(1,  [0.0, 0.017, 0.663, 0.873]))
        step5.update(self.finger_pose(5,  [-0.25, 0.0, 0.0, 0.0]))
        step5.update(self.finger_pose(9,  [-0.1, 0.0, 0.0, 0.0]))
        step5.update(self.finger_pose(13, [0.1, 0.0, 0.0, 0.0]))
        step5.update(self.finger_pose(17, [0.25, 0.0, 0.0, 0.0]))
        traj_steps.append(step5)

        traj_steps.append(release_pose)

        step6 = release_pose.copy()
        step6.update(self.finger_pose(1,  [0.0, 0.035, 0.415, 1.03]))
        step6.update(self.finger_pose(5,  [-0.42, 0.0, 0.0, 0.0]))
        step6.update(self.finger_pose(9,  [-0.15, 0.0, 0.0, 0.0]))
        step6.update(self.finger_pose(13, [0.15, 0.0, 0.0, 0.0]))
        step6.update(self.finger_pose(17, [0.42, 0.0, 0.0, 0.0]))
        traj_steps.append(step6)

        
        # traj_steps.append(base_set)

        # -------------------------     각 finger 0번 joint들은 아직 조정 안 함.
        # finger2
        step1_1 = base_set.copy()
        step1_1.update(self.finger_pose(5, common_pose1))
        step1_1.update(self.finger_pose(1, [0.804, -0.278, 0.294, 0.742]))   # +0.1
        traj_steps.append(step1_1)

        step1_2 = base_set.copy()
        step1_2.update(self.finger_pose(5, common_pose1))
        step1_2.update(self.finger_pose(1, [0.753, -0.581, 0.283, 0.708]))
        traj_steps.append(step1_2)

        step1_3 = base_set.copy()
        step1_3.update(self.finger_pose(5, common_pose1))
        step1_3.update(self.finger_pose(1, [0.787, -0.992, -0.500, 1.570]))
        traj_steps.append(step1_3)

        # -------------------------
        # finger3 
        step2_1 = base_set.copy()
        step2_1.update(self.finger_pose(9, common_pose2))    #  여기 이거 체크 하기 : 0번 joint들은 다른 값 들어가야 함.
        step2_1.update(self.finger_pose(1, [1.281, -0.192, 0.317, 0.272]))   # +0.05
        traj_steps.append(step2_1)

        step2_2 = base_set.copy()
        step2_2.update(self.finger_pose(9, common_pose2))
        step2_2.update(self.finger_pose(1, [0.822, -0.711, 0.686, 0.440]))
        traj_steps.append(step2_2)

        step2_3 = base_set.copy()
        step2_3.update(self.finger_pose(9, common_pose2))
        step2_3.update(self.finger_pose(1, [0.873, -1.057, -0.008, 1.357]))
        traj_steps.append(step2_3)

        # -------------------------
        # finger4
        step3_1 = base_set.copy()
        step3_1.update(self.finger_pose(13, common_pose3))
        step3_1.update(self.finger_pose(1, [1.57, -0.192, 0.311, 0.272]))  # 3번 joint : +0.05
        traj_steps.append(step3_1)

        step3_2 = base_set.copy()
        step3_2.update(self.finger_pose(13, common_pose3))
        step3_2.update(self.finger_pose(1, [1.57, -0.473, 0.289, 0.272]))
        traj_steps.append(step3_2)

        step3_3 = base_set.copy()
        step3_3.update(self.finger_pose(13, common_pose3))
        step3_3.update(self.finger_pose(1, [1.332, -0.970, 0.199, 0.966]))
        traj_steps.append(step3_3)

        # -------------------------
        # finger5
        step4_1 = base_set.copy()
        step4_1.update(self.finger_pose(17, [0.2, 1.622, 1.095, 0.823]))
        step4_1.update(self.finger_pose(1, [0.921, -1.381, 1.57, 0.00]))  # 1번 조인트 + 0.03
        traj_steps.append(step4_1)

        step4_2 = base_set.copy()
        step4_2.update(self.finger_pose(17, [0.2, 1.622, 1.095, 0.823]))
        step4_2.update(self.finger_pose(1, [1.294, -1.165, 0.697, 0.876]))
        traj_steps.append(step4_2)

        step4_3 = base_set.copy()
        step4_3.update(self.finger_pose(17, [0.2, 1.622, 1.095, 0.823]))
        step4_3.update(self.finger_pose(1, [1.57, -1.64, 0.507, 1.57]))
        traj_steps.append(step4_3)

        traj_steps.append(base_set)

        traj_steps.append(base_set)
        traj_steps.append(base_set)

        # traj_steps.append(grisp)
        # traj_steps.append(grisp)

        return traj_steps

    def timer_callback(self):
        target_pose = self.traj_steps[self.current_step_idx]

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [target_pose[joint] for joint in self.joint_names]

        # 이동시간 짧게
        sec = int(self.move_time)
        nanosec = int((self.move_time - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec

        msg.points.append(point)
        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'Published step {self.current_step_idx + 1}/{len(self.traj_steps)}'
        )

        self.current_step_idx = (self.current_step_idx + 1) % len(self.traj_steps)


def main(args=None):
    rclpy.init(args=args)
    node = HandStepControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down by Ctrl+C')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()