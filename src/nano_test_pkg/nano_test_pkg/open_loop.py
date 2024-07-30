#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from px4_msgs.msg import OffboardControlMode, VehicleCommand, TrajectorySetpoint

# Note: This is just a simple implementation of a node that arms the simulation drone (no timers, no switching between arming/disarming, etc)

class ArmDisarmNode(Node):
    def __init__(self):
        super().__init__("arm_disarm")
        self.offboard_publisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.arm_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.counter = 0
        self.timer_ = self.create_timer(0.01, self.main_func)
    def main_func(self):
        if self.counter == 150:
            self.send_arm_command()
        if self.counter == 301:
            # self.send_mode_command()
            self.send_takeoff_command()
        # self.ocm()
        # self.send_arm_command() #Code to call send_arm_command every few seconds
            
        
        if self.counter < 301:
            self.counter += 1
        print(self.counter)
    def ocm(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position = [0.0, 0.0, -2.0]
        trajectory_msg.yaw = 0.0
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_publisher_.publish(trajectory_msg)
        self.offboard_publisher_.publish(offboard_msg)

        print("publishing offboard control mode")
    def send_mode_command(self):
        mode_msg = VehicleCommand()
        mode_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        mode_msg.param1 = 1.0
        mode_msg.param2 = 6.0
        # Idk if we need the rest of this stuff

        mode_msg.target_system = 1
        mode_msg.target_component = 1
        mode_msg.source_system = 1
        mode_msg.source_component = 1
        mode_msg.from_external = True
        mode_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.arm_publisher_.publish(mode_msg)
    def send_arm_command(self):
        # self.arm_toggle_ = not self.arm_toggle_ # Code to switch between arming/disarming nodes 
    
        arm_msg = VehicleCommand()
        arm_msg.param1 = 1.0
        arm_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        arm_msg.target_system  = 1
        arm_msg.target_component = 1
        arm_msg.source_system = 1
        arm_msg.source_component = 1
        arm_msg.from_external = True
        arm_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.arm_publisher_.publish(arm_msg)

        print("Arm toggle message sent")
    def send_takeoff_command(self):
        # self.arm_toggle_ = not self.arm_toggle_ # Code to switch between arming/disarming nodes 
    
        msg = VehicleCommand()
        msg.param1 = 0.0
        msg.param7 = 2.0
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        msg.target_system  = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        # msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.arm_publisher_.publish(msg)

        print("Arm toggle message sent")


def main(args=None):
    rclpy.init(args=args)
    node = ArmDisarmNode()
    # node.send_arm_command() # Sends the command once
    rclpy.spin(node)
    rclpy.shutdown()

# Other stuff:
# # Open-Loop control portion:
#     '''
#     while True:
#         key = node.getKey(settings)
#         if key == ' ':
#             node.send_arm_command()
#     '''
# # Code to switch between arming/disarming mode
#     '''
#     if self.arm_toggle_:
#         arm_msg.param1 = 1.0
#     else:
#         arm_msg.param1 = 0.0
#     '''







# import rclpy
# from rclpy.node import Node
# import px4_msgs.msg

# # Note: This is just a simple implementation of a node that arms the simulation drone (no timers, no switching between arming/disarming, etc)

# class ArmDisarmNode(Node):
#     def __init__(self):
#         super().__init__("arm_disarm")
#         self.arm_toggle_ = False # Not needed for this
#         self.arm_publisher_ = self.create_publisher(px4_msgs.msg.VehicleCommand, '/fmu/in/vehicle_command', 10)
#         # self.timer_ = self.create_timer(5, self.send_arm_command) Code to call send_arm_command every few seconds
        
    
#     def send_arm_command(self):
#         self.arm_toggle_ = not self.arm_toggle_ # Code to switch between arming/disarming nodes 
#         arm_msg = px4_msgs.msg.VehicleCommand()
#         arm_msg.command = px4_msgs.msg.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
#         arm_msg.param1 = 1.0

#         # Idk if we need the rest of this stuff
#         arm_msg.target_system = 1
#         arm_msg.target_component = 1
#         arm_msg.source_system = 1
#         arm_msg.source_component = 1
#         arm_msg.from_external = True

#         self.arm_publisher_.publish(arm_msg)
#         print("Arm toggle message sent")


# def main(args=None):
#     rclpy.init(args=args)
#     node = ArmDisarmNode()
#     node.send_arm_command() # Sends the command once
#     # rclpy.spin(node)
#     rclpy.shutdown()

# # Other stuff:
# # Open-Loop control portion:
#     '''
#     while True:
#         key = node.getKey(settings)
#         if key == ' ':
#             node.send_arm_command()
#     '''
# # Code to switch between arming/disarming mode
#     '''
#     if self.arm_toggle_:
#         arm_msg.param1 = 1.0
#     else:
#         arm_msg.param1 = 0.0
#     '''



# #!usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from rclpy.clock import Clock
# from px4_msgs.msg import VehicleCommand
# from drone_msgs.srv import ArmDisarm

# class ArmingNode(Node):
#     def __init__(self):
#         # the below contains the name of the node in "name"
#         super().__init__("arm_disarm_node")
#         self.publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
#         self.srv = self.create_service(ArmDisarm, 'arm_disarm', self.callback)
#     def callback(self, request, response):
#         if request.should_arm:
#             self.arm()
#         else:
#             self.disarm()

#         response.new_state = request.should_arm
#         return response

#     def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
#         msg = VehicleCommand()
#         msg.param1 = param1
#         msg.param2 = param2
#         msg.param7 = param7    # altitude value in takeoff command
#         msg.command = command  # command ID
#         msg.target_system = 1  # system which should execute the command
#         msg.target_component = 1  # component which should execute the command, 0 for all components
#         msg.source_system = 1  # system sending the command
#         msg.source_component = 1  # component sending the command
#         msg.from_external = True
#         msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
#         self.publisher.publish(msg)

#     def arm(self):
#         self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
#         self.get_logger().info("Arm command sent")

#     def disarm(self):
#         self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
#         self.get_logger().info("Disarm command sent")

# def main(args=None):
#     rclpy.init(args=args)
#     arm_disarm_node = ArmingNode()
#     rclpy.spin(arm_disarm_node)
#     rclpy.shutdown()