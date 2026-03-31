import json
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import ThreadingOSCUDPServer


class VMCReceiverNode(Node):
    def __init__(self):
        super().__init__("vmc_receiver_node")

        self.declare_parameter("listen_ip", "0.0.0.0")
        self.declare_parameter("listen_port", 39539)
        self.declare_parameter("world_frame", "vmc_world")
        self.declare_parameter("publish_only_hands", False)

        self.listen_ip = self.get_parameter("listen_ip").get_parameter_value().string_value
        self.listen_port = self.get_parameter("listen_port").get_parameter_value().integer_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.publish_only_hands = self.get_parameter("publish_only_hands").get_parameter_value().bool_value

        self.raw_pub = self.create_publisher(String, "/udcap/vmc/raw", 1000)
        self.tf_pub = self.create_publisher(TFMessage, "/udcap/vmc/tf", 1000)

        self.hand_keywords = [
            "Thumb", "Index", "Middle", "Ring", "Little",
            "Wrist", "Hand"
        ]

        dispatcher = Dispatcher()
        dispatcher.map("/VMC/Ext/Bone/Pos", self.handle_bone_pos)
        dispatcher.set_default_handler(self.handle_default)

        self.server = ThreadingOSCUDPServer((self.listen_ip, self.listen_port), dispatcher)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()

        self.get_logger().info(
            f"Listening VMC on {self.listen_ip}:{self.listen_port}"
        )

        self.right_hand_rotation_pubs = {}

    def bone_to_topic_name(self, bone_name: str) -> str:
        # RightThumbProximal -> right_thumb_proximal
        result = []
        for i, ch in enumerate(bone_name):
            if ch.isupper() and i > 0:
                result.append("_")
            result.append(ch.lower())
        return "".join(result)

    def publish_right_hand_rotation(self, bone_name: str, qx: float, qy: float, qz: float, qw: float):
        if not bone_name.startswith("Right"):
            return

        if not any(k in bone_name for k in ["Hand", "Thumb", "Index", "Middle", "Ring", "Little"]):
            return

        bone_topic_name = self.bone_to_topic_name(bone_name)
        topic_name = f"/udcap/vmc/right_hand/{bone_topic_name}/rotation"

        if topic_name not in self.right_hand_rotation_pubs:
            self.right_hand_rotation_pubs[topic_name] = self.create_publisher(String, topic_name, 100)

        msg = String()
        msg.data = json.dumps({
            "x": qx,
            "y": qy,
            "z": qz,
            "w": qw,
        })
        self.right_hand_rotation_pubs[topic_name].publish(msg)

    def is_hand_bone(self, bone_name: str) -> bool:
        return any(k in bone_name for k in self.hand_keywords)

    def handle_bone_pos(self, address, *args):
        if len(args) < 8:
            self.get_logger().warn(f"Invalid Bone/Pos args: {args}")
            return

        bone_name = str(args[0])

        if self.publish_only_hands and not self.is_hand_bone(bone_name):
            return

        try:
            x = float(args[1])
            y = float(args[2])
            z = float(args[3])
            qx = float(args[4])
            qy = float(args[5])
            qz = float(args[6])
            qw = float(args[7])
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f"Failed to parse bone data: {bone_name}, err={e}")
            return

        # if any(k in bone_name for k in ["Thumb", "Index", "Middle", "Ring", "Little"]):
        #     self.get_logger().info(
        #         f"{bone_name} "
        #         f"pos=({x:.3f}, {y:.3f}, {z:.3f}) "
        #         f"rot=({qx:.5f}, {qy:.5f}, {qz:.5f}, {qw:.5f})"
        #     )

        if bone_name.startswith("Right") and any(k in bone_name for k in [
            "Thumb"#, "Index", "Middle", "Ring", "Little", "Hand"
        ]):
            self.get_logger().info(
                f"{bone_name} "
                f"pos=({x:.3f}, {y:.3f}, {z:.3f}) "
                f"rot=({qx:.5f}, {qy:.5f}, {qz:.5f}, {qw:.5f})"
            )

        now = self.get_clock().now().to_msg()

        raw_msg = String()
        raw_msg.data = json.dumps({
            "address": address,
            "bone_name": bone_name,
            # "position": {"x": x, "y": y, "z": z},
            "rotation": {"x": qx, "y": qy, "z": qz, "w": qw},   # "x": qx, "y": qy,
        })
        self.raw_pub.publish(raw_msg)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.world_frame
        tf.child_frame_id = bone_name

        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z

        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        tf_msg = TFMessage()
        tf_msg.transforms.append(tf)
        self.tf_pub.publish(tf_msg)
        self.publish_right_hand_rotation(bone_name, qx, qy, qz, qw)  # right pub

    def handle_default(self, address, *args):
        # 디버깅용: Bone/Pos 이외 메시지도 raw로 확인 가능
        raw_msg = String()
        raw_msg.data = json.dumps({
            "address": address,
            "args": [str(a) for a in args]
        })
        self.raw_pub.publish(raw_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VMCReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()