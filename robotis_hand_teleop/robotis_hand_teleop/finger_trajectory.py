#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HandPosePublisher(Node):
    def __init__(self):
        super().__init__("hand_pose_publisher")

        self.pub = self.create_publisher(
            JointTrajectory,
            "/left_hand_controller/joint_trajectory",
            10,
        )

        # ====== Timing ======
        # self.stage_dt = 0.2
        # self.start_delay = 0.15
        # self.interp_steps = 7
        # self.repeat_count = 2
        # self.final_home_dt = 0.2

        self.stage_dt = 1.0
        # self.start_delay = 1.0
        # self.interp_steps = 2
        # self.repeat_count = 1
        # self.final_home_dt = 1.0

        self.repeat_count = 1
        self.interp_steps = 1
        self.start_delay = 0.0
        self.final_home_dt = 0.0

        self._done = False
        self.timer = self.create_timer(0.2, self.publish_once)
        # self.timer = self.create_timer(self.stage_dt, self.publish_loop)

    # -------------------------
    # Utils
    # -------------------------
    @staticmethod
    def all_joint_names():
        return [f"finger_l_joint{i}" for i in range(1, 21)]

    @staticmethod
    def base_positions_dict(val: float = 0.0):
        return {f"finger_l_joint{i}": val for i in range(1, 21)}

    @staticmethod
    def make_point(positions_list, t_from_start: float):
        pt = JointTrajectoryPoint()
        pt.positions = positions_list
        sec = int(t_from_start)
        nsec = int((t_from_start - sec) * 1e9)
        pt.time_from_start.sec = sec
        pt.time_from_start.nanosec = nsec
        return pt

    @staticmethod
    def lerp(a, b, u):
        return a + (b - a) * u

    def dict_lerp(self, d0, d1, u):
        keys = set(d0) | set(d1)
        out = {}
        for k in keys:
            v0 = float(d0.get(k, 0.0))
            v1 = float(d1.get(k, 0.0))
            out[k] = self.lerp(v0, v1, u)
        return out

    def append_segment(self, traj, joint_names, pose_a, pose_b, t_start, duration):
        steps = max(2, int(self.interp_steps))
        dt = duration / steps
        for i in range(1, steps + 1):
            u = i / steps
            pose = self.dict_lerp(pose_a, pose_b, u)
            t = t_start + dt * i
            traj.points.append(self.make_point([pose[j] for j in joint_names], t))

    # degree -> rad 변환해서 finger joint dict 생성
    @staticmethod
    def finger_pose(start_joint_idx: int, deg_vals):
        return {
            f"finger_l_joint{start_joint_idx + 0}": math.radians(deg_vals[0]),
            f"finger_l_joint{start_joint_idx + 1}": math.radians(deg_vals[1]),
            f"finger_l_joint{start_joint_idx + 2}": math.radians(deg_vals[2]),
            f"finger_l_joint{start_joint_idx + 3}": math.radians(deg_vals[3]),
        }

    # base pose 위에 step pose를 덮어쓰는 방식
    def build_stage_pose(self, base_pose, overrides):
        pos = dict(base_pose)
        pos.update(overrides)
        return pos

    def build_full_trajectory(self) -> JointTrajectory:

        joint_names = self.all_joint_names()
        traj = JointTrajectory()
        traj.header.stamp = (self.get_clock().now() + rclpy.duration.Duration(seconds=0.3)).to_msg()
        traj.joint_names = joint_names

        # =========================
        # 기본 자세
        # Finger 1 ~ 5
        # =========================
        home_pose = self.base_positions_dict(0.0)
        home_pose.update(self.finger_pose(1,  [-40.6, 59.2, 29.5, -48.3]))  # Finger 1
        home_pose.update(self.finger_pose(5,  [24.3, 51.9, 8.4, 41.2]))     # Finger 2
        home_pose.update(self.finger_pose(9,  [2.8, 33.8, 17.1, 39.0]))     # Finger 3
        home_pose.update(self.finger_pose(13, [-12.7, 44.0, 8.3, 38.7]))    # Finger 4
        home_pose.update(self.finger_pose(17, [-20.4, 35.7, 26.1, 38.2]))   # Finger 5

        # =========================
        step1 = {}
        step1.update(self.finger_pose(1,  [-33.4, 59.2, -22.9, -21.4]))  # Finger 1
        step1.update(self.finger_pose(5,  [24.2, 60.4, 17.2, 40.7]))     # Finger 2

        step2 = {}
        step2.update(self.finger_pose(1,  [-18.2, 107.9, -8.3, -91.6]))  # Finger 1
        step2.update(self.finger_pose(9,  [2.8, 48.0, 83.7, 28.2]))      # Finger 3

        step3 = {}
        step3.update(self.finger_pose(1,  [-25.9, 116.3, -8.3, -91.6]))  # Finger 1
        step3.update(self.finger_pose(13, [-14.5, 54.5, 71.3, 29.4]))    # Finger 4

        step4 = {}
        step4.update(self.finger_pose(1,  [-51.4, 131.0, -30.0, -60.2])) # Finger 1
        step4.update(self.finger_pose(17, [-17.8, 62.6, 71.4, 54.0]))    # Finger 5

        stages = [
            ("finger1+finger2", step1),
            ("finger1+finger3", step2),
            ("finger1+finger4", step3),
            ("finger1+finger5", step4),
        ]

        # 왕복 시퀀스
        seq = [
            stages[0], stages[1], stages[2], stages[3],
            stages[2], stages[1], stages[0],
        ]

        t = 0.0
        current = home_pose

        # 시작 전 기본 자세 유지
        self.append_segment(traj, joint_names, current, current, t_start=t, duration=self.start_delay)
        t += self.start_delay

        # 첫 step
        _, first_override = seq[0]
        first_pose = self.build_stage_pose(home_pose, first_override)
        self.append_segment(traj, joint_names, current, first_pose, t_start=t, duration=self.stage_dt)
        t += self.stage_dt
        current = first_pose

        # 반복
        for _ in range(self.repeat_count):
            for _, override in seq[1:]:
                nxt = self.build_stage_pose(home_pose, override)
                self.append_segment(traj, joint_names, current, nxt, t_start=t, duration=self.stage_dt)
                t += self.stage_dt
                current = nxt

        # 마지막 home 복귀
        self.append_segment(traj, joint_names, current, home_pose, t_start=t, duration=self.final_home_dt)

        self.get_logger().info(
            f"Built trajectory: repeat={self.repeat_count}, points={len(traj.points)}"
        )
        return traj

    def publish_once(self):
        if self._done:
            return
        self._done = True

        traj = self.build_full_trajectory()
        self.pub.publish(traj)

        self.get_logger().info("Published full motion (HOME -> repeats -> HOME). Shutting down...")
        self.timer.cancel()
        self.shutdown_timer = self.create_timer(3.0, self.delayed_shutdown)

    def delayed_shutdown(self):
        self.get_logger().info("Shutting down...")
        self.shutdown_timer.cancel()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = HandPosePublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()