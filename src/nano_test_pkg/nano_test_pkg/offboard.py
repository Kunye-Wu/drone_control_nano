#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from px4_msgs.msg import OffboardControlMode, VehicleCommand, TrajectorySetpoint
from keyboard_msgs.msg import Key


class Offboard(Node):
    def __init__(self):
        super().__init__("offboard")
        self.vc_publisher_ = self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command', 10)
        self.offboard_publisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.trajectory_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.key_subscriber_ = self.create_subscription(Key, "/keydown", self.main_func, 10)
        self.armed = 1.0
        self.x = 0.0
        self.y = 0.0
        self.z = -1.0
        self.timer_ = self.create_timer(0.01, self.trajocm)
    def trajocm(self):
        self.traj(x = self.x, y = self.y, z = self.z)
        self.ocm()
    def main_func(self, msg: Key):
        if msg.code == 119: #set mode offboard
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        elif msg.code == 97: #arm disarm
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1= self.armed)
            if self.armed == 0.0:
                self.armed = 1.0
            else:
                self.armed = 0.0
            self.get_logger().info(str(self.armed))
        elif msg.code == 115: #manual mode
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2 = 1.0)
        elif msg.code == 121: #go up
            self.z -= 0.1
            self.get_logger().info(str(self.x) + " " + str(self.y) + " " + str(self.z))
        elif msg.code == 104: #go down
            self.z += 0.1
            self.get_logger().info(str(self.x) + " " + str(self.y) + " " + str(self.z))
        elif msg.code == 105: #go forward
            self.x += 0.1
            self.get_logger().info(str(self.x) + " " + str(self.y) + " " + str(self.z))
        elif msg.code == 107: #go backward
            self.x -= 0.1
            self.get_logger().info(str(self.x) + " " + str(self.y) + " " + str(self.z))
        elif msg.code == 106: #go left
            self.y -= 0.1
            self.get_logger().info(str(self.x) + " " + str(self.y) + " " + str(self.z))
        elif msg.code == 108: #go right
            self.y += 0.1
            self.get_logger().info(str(self.x) + " " + str(self.y) + " " + str(self.z))
            
    def ocm(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.offboard_publisher_.publish(offboard_msg)
        # self.get_logger().info("publishing offboard control mode")
    def traj(self, x=0.0, y=0.0, z=0.0):
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position = [x, y, z]
        trajectory_msg.yaw = -3.1415
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_publisher_.publish(trajectory_msg)
        # self.get_logger().info("trajectory setpoint")
    def send_vehicle_command(self, command, param1 = 0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vc_publisher_.publish(msg)
        self.get_logger().info(str(command) + " published to")

def main(args=None):
    rclpy.init(args=args)
    node = Offboard()
    rclpy.spin(node)
    rclpy.shutdown()