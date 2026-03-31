import json
from typing import Dict

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class VmcToRightHandTrajectory(Node):
    def __init__(self):
        super().__init__("vmc_to_right_hand_trajectory")
        
        self.sub = self.create_subscription(
            String,
            "/udcap/vmc/raw",
            self.raw_callback,
            100,
        )
        self.pub = self.create_publisher(
            JointTrajectory,
            "/right_hand_controller/joint_trajectory",  # "/right_hand_controller/joint_trajectory"    "/joint_states"
            10,
        )

        self.joint_order = [
            "finger_r_joint1", "finger_r_joint2", "finger_r_joint3", "finger_r_joint4",
            "finger_r_joint5", "finger_r_joint6", "finger_r_joint7", "finger_r_joint8",
            "finger_r_joint9", "finger_r_joint10", "finger_r_joint11", "finger_r_joint12",
            "finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16",
            "finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20",
        ]

        self.bone_to_joint = {
            "RightThumbProximal": "finger_r_joint1",
            "RightThumbIntermediate": "finger_r_joint3",
            "RightThumbDistal": "finger_r_joint4",

            "RightIndexProximal": "finger_r_joint6",
            "RightIndexIntermediate": "finger_r_joint7",
            "RightIndexDistal": "finger_r_joint8",

            "RightMiddleProximal": "finger_r_joint10",
            "RightMiddleIntermediate": "finger_r_joint11",
            "RightMiddleDistal": "finger_r_joint12",

            "RightRingProximal": "finger_r_joint14",
            "RightRingIntermediate": "finger_r_joint15",
            "RightRingDistal": "finger_r_joint16",

            "RightLittleProximal": "finger_r_joint18",
            "RightLittleIntermediate": "finger_r_joint19",
            "RightLittleDistal": "finger_r_joint20",
        }

        self.proximal_y_to_joint = {
            "RightIndexProximal": "finger_r_joint5",
            "RightMiddleProximal": "finger_r_joint9",
            "RightRingProximal": "finger_r_joint13",
            "RightLittleProximal": "finger_r_joint17",
        }

        self.thumb_axis_to_joint = {
            ("RightThumbProximal", "x"): "finger_r_joint1",
            ("RightThumbProximal", "y"): "finger_r_joint2",
            ("RightThumbIntermediate", "y"): "finger_r_joint3",
            ("RightThumbDistal", "y"): "finger_r_joint4",
        }

        self.joint_min = {joint: 0.0 for joint in self.joint_order}
        self.joint_max = {joint: 1.5 for joint in self.joint_order}

        self.current_positions: Dict[str, float] = {
            joint: 0.0 for joint in self.joint_order
        }

        self.publish_period = 0.05
        self.timer = self.create_timer(self.publish_period, self.publish_trajectory)

        self.get_logger().info("VMC qz -> right_hand_controller bridge started")

    def map_qz_to_joint(self, qz: float) -> float:
        # 입력:  0.0  ~ -0.64
        # 출력:  0.0  ~  1.5
        angle = (-qz / 0.64) * 1.5
        return clamp(angle, 0.0, 1.5)

    def map_qy_proximal_to_joint(self, qy: float) -> float:
        # 입력 최대 0.3 -> 출력 최대 0.5
        angle = (qy / 0.3) * 0.5
        return clamp(angle, -0.5, 0.5)

    def map_thumb_axis_to_joint(self, value: float) -> float:
        # 입력 최대 0.5 -> 출력 최대 0.5
        angle = (value / 0.5) * 1.5
        return clamp(angle, 0.0, 1.5)

    def raw_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if data.get("address") != "/VMC/Ext/Bone/Pos":
            return

        bone_name = data.get("bone_name", "")
        if bone_name not in self.bone_to_joint:
            return

        rot = data.get("rotation", {})
        try:
            qx = float(rot["x"])
            qy = float(rot["y"])
            qz = float(rot["z"])
        except Exception:
            return

        # 기존: z값 -> 기존 joint
        joint_name = self.bone_to_joint[bone_name]
        angle = self.map_qz_to_joint(qz)
        self.current_positions[joint_name] = angle

        # 추가: Proximal bone의 y값 -> 추가 joint
        if bone_name in self.proximal_y_to_joint:
            proximal_y_joint = self.proximal_y_to_joint[bone_name]
            proximal_y_angle = self.map_qy_proximal_to_joint(qy)
            self.current_positions[proximal_y_joint] = proximal_y_angle
        
            self.get_logger().info(
                    f"{bone_name} qy={qy:.5f} -> {proximal_y_joint}={proximal_y_angle:.5f}, "
                    f"qz={qz:.5f} -> {joint_name}={angle:.5f}"
                )
        
        if (bone_name, "x") in self.thumb_axis_to_joint:
            thumb_x_joint = self.thumb_axis_to_joint[(bone_name, "x")]
            thumb_x_angle = self.map_thumb_axis_to_joint(qx)
            self.current_positions[thumb_x_joint] = thumb_x_angle

        if (bone_name, "y") in self.thumb_axis_to_joint:
            thumb_y_joint = self.thumb_axis_to_joint[(bone_name, "y")]
            y_value = -qy if bone_name == "RightThumbProximal" else qy
            thumb_y_angle = self.map_thumb_axis_to_joint(y_value)
            self.current_positions[thumb_y_joint] = thumb_y_angle

        else:
            self.get_logger().info(
                f"{bone_name} qz={qz:.5f} -> {joint_name}={angle:.5f}"
            )

    def publish_trajectory(self):
        traj = JointTrajectory()
        traj.header.stamp.sec = 0
        traj.header.stamp.nanosec = 0
        traj.header.frame_id = ""
        traj.joint_names = self.joint_order

        point = JointTrajectoryPoint()
        point.positions = [self.current_positions[j] for j in self.joint_order]
        point.time_from_start = Duration(sec=0, nanosec=10000000)

        traj.points.append(point)
        self.pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = VmcToRightHandTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()