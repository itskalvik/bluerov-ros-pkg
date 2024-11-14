#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from pymavlink import mavutil
import sys
import os

class LightControlService(Node):
    def __init__(self):
        super().__init__('light_control_service')
        self.srv = self.create_service(SetBool, 'light_control', self.handle_light_control)
        self.get_logger().info("Light control service is ready.")

        # Check if running on NUC without Pixhawk
        if not os.path.exists('/dev/ttyACM0'):
            self.get_logger().warning("Pixhawk not connected. Running in test mode.")
            self.master = None  # No Pixhawk connection
        else:
            # Attempt to connect to Pixhawk if present
            self.get_logger().info("Attempting to connect to Pixhawk on /dev/ttyACM0") 
            try:
                self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
                self.get_logger().info("Connection object created; waiting for heartbeat.") 
                self.master.wait_heartbeat(timeout=10)
                self.get_logger().info("Connected to Pixhawk")
            except Exception as e:
                self.get_logger().error(f"Failed to connect to Pixhawk: {e}")
                sys.exit(1)

    def set_pwm(self, channel, pwm_value):
        """Function to set PWM value if Pixhawk is connected."""
        if self.master:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,               # confirmation
                channel,         # param1 (servo channel)
                pwm_value,       # param2 (PWM value)
                0, 0, 0, 0, 0    # param3-param7 (unused)
            )
        else:
            self.get_logger().info(f"Test mode: would set PWM on channel {channel} to {pwm_value}")

    def handle_light_control(self, request, response):
        """Service callback to turn the light on or off."""
        if request.data:
            try:
                self.set_pwm(9, 1500)  # Turn the light on
                response.success = True
                response.message = "Light turned on"
            except Exception as e:
                response.success = False
                response.message = f"Failed to turn on the light: {e}"
        else:
            try:
                self.set_pwm(9, 1100)  # Turn the light off
                response.success = True
                response.message = "Light turned off"
            except Exception as e:
                response.success = False
                response.message = f"Failed to turn off the light: {e}"
        return response

def main(args=None):
    rclpy.init(args=args)
    light_control_service = LightControlService()
    rclpy.spin(light_control_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

