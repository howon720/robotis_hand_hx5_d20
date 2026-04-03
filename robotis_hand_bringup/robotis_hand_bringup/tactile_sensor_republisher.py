#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState
from robotis_interfaces.msg import TactileSensor, Sensor


class TactileSensorRepublisher(Node):
    def __init__(self):
        super().__init__('tactile_sensor_republisher')

        self.declare_parameter('input_topic', '/dynamic_joint_states')
        self.declare_parameter('output_topic', '/tactile_sensor')
        self.declare_parameter('sensor_prefix', 'finger_r_sensor')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.sensor_prefix = self.get_parameter('sensor_prefix').get_parameter_value().string_value

        self.sub = self.create_subscription(
            DynamicJointState,
            input_topic,
            self.callback,
            10,
        )
        self.pub = self.create_publisher(TactileSensor, output_topic, 10)

        self.get_logger().info(f'Subscribing: {input_topic}')
        self.get_logger().info(f'Publishing  : {output_topic}')
        self.get_logger().info(f'Sensor prefix: {self.sensor_prefix}')

    def callback(self, msg: DynamicJointState):
        out = TactileSensor()
        out.header = msg.header

        for joint_name, interface_block in zip(msg.joint_names, msg.interface_values):
            if not joint_name.startswith(self.sensor_prefix):
                continue

            pressure_map = {}
            for iface_name, value in zip(interface_block.interface_names, interface_block.values):
                pressure_map[iface_name] = value

            unit = Sensor()
            unit.sensor_name = joint_name
            unit.pressure_1 = float(pressure_map.get('Present Pressure 1', 0.0))
            unit.pressure_2 = float(pressure_map.get('Present Pressure 2', 0.0))
            unit.pressure_3 = float(pressure_map.get('Present Pressure 3', 0.0))
            unit.pressure_4 = float(pressure_map.get('Present Pressure 4', 0.0))
            unit.pressure_5 = float(pressure_map.get('Present Pressure 5', 0.0))
            unit.pressure_6 = float(pressure_map.get('Present Pressure 6', 0.0))
            unit.pressure_7 = float(pressure_map.get('Present Pressure 7', 0.0))
            unit.pressure_8 = float(pressure_map.get('Present Pressure 8', 0.0))
            unit.pressure_9 = float(pressure_map.get('Present Pressure 9', 0.0))

            out.sensors.append(unit)

        if len(out.sensors) == 0:
            return

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TactileSensorRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()