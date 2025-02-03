#!/usr/bin/env python3
# light_service_node.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from mavros_msgs.msg import OverrideRCIn
import sys

class LightControlService(Node):
    def __init__(self):
        super().__init__('light_control_service')

        # Declare parameters with default values
        self.declare_parameter('light_channel', 9)
        self.declare_parameter('light_on_value', 1500)
        self.declare_parameter('light_off_value', 1100)

        # Retrieve parameters
        self.light_channel = self.get_parameter('light_channel').get_parameter_value().integer_value
        self.light_on_value = self.get_parameter('light_on_value').get_parameter_value().integer_value
        self.light_off_value = self.get_parameter('light_off_value').get_parameter_value().integer_value

        # Create the service
        self.srv = self.create_service(SetBool, 'light_control', self.handle_light_control)
        self.get_logger().info("Light control service is ready.")

        # Publisher to /mavros/rc/override
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # Initialize OverrideRCIn message
        self.rc_override_msg = OverrideRCIn()
        self.rc_override_msg.channels = [0] * 18  # Initialize all channels to 0 (no override)
        self.override_rc(self.light_channel, self.light_off_value)  # Set initial state to off

        # Create a timer to publish overrides at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_override)

        self.get_logger().info("RC Override Publisher initialized.")

    def handle_light_control(self, request, response):
        if request.data:
            try:
                self.override_rc(self.light_channel, self.light_on_value)
                response.success = True
                response.message = "Light turned on"
                self.get_logger().info("Light turned on via RC override.")
            except Exception as e:
                response.success = False
                response.message = f"Failed to turn on the light: {e}"
                self.get_logger().error(f"Failed to turn on the light: {e}")
        else:
            try:
                self.override_rc(self.light_channel, self.light_off_value)
                response.success = True
                response.message = "Light turned off"
                self.get_logger().info("Light turned off via RC override.")
            except Exception as e:
                response.success = False
                response.message = f"Failed to turn off the light: {e}"
                self.get_logger().error(f"Failed to turn off the light: {e}")
        return response

    def override_rc(self, channel, value):
        # Validate channel number
        if not (1 <= channel <= 18):
            self.get_logger().error(f"Invalid RC channel: {channel}. Must be between 1 and 18.")
            return
        self.rc_override_msg.channels[channel - 1] = value
        self.get_logger().debug(f"Set RC Override on channel {channel}: {value}")

    def publish_override(self):
        self.rc_override_pub.publish(self.rc_override_msg)
        self.get_logger().debug(f"Publishing RC Override: {self.rc_override_msg.channels}")

def main(args=None):
    rclpy.init(args=args)
    light_control_service = LightControlService()
    try:
        rclpy.spin(light_control_service)
    except KeyboardInterrupt:
        light_control_service.get_logger().info("Light control service stopped by user.")
    finally:
        light_control_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

