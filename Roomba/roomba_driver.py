from key_commander import KeyCommander
from pynput.keyboard import KeyCode

import math
from threading import Lock
import time

import rclpy
from rclpy.action.client import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from action_msgs.msg._goal_status import GoalStatus
from irobot_create_msgs.action import Undock, Dock, DriveDistance, RotateAngle
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import String
#from tf_transformations import euler_from_quaternion

lock = Lock()

cobot_name = "velma"
roomba_name = "create3_03EE"

class RoombaDriver(Node):
    def __init__(self):
        super().__init__('roomba_driver')

        self._current_pose = None
        self._dock_position = None
        self._goal_uuid = None
        cb_Navigation = MutuallyExclusiveCallbackGroup()
        cb_Action = MutuallyExclusiveCallbackGroup()
        cb_Communication = MutuallyExclusiveCallbackGroup()

        self._undock_action_client = ActionClient (
            self, 
            Undock, 
            f'/{roomba_name}/undock', 
            callback_group = cb_Action
        )
        self._dock_action_client = ActionClient (
            self, 
            Dock, 
            f'/{roomba_name}/dock', 
            callback_group = cb_Action
        )
        self._drive_action_client = ActionClient (
            self, 
            DriveDistance, 
            f'/{roomba_name}/drive_distance', 
            callback_group = cb_Action
        )
        self._rotate_action_client = ActionClient (
            self, 
            RotateAngle, 
            f'/{roomba_name}/rotate_angle', 
            callback_group = cb_Action
        )

        self._publisher = self.create_publisher (
            String,
            f'/{roomba_name}/message_roomba',
            10
        )

        self._odometer_subscription = self.create_subscription (
            Odometry,
            f'/{roomba_name}/odom',
            self.odometer_callback,
            qos_profile_sensor_data,
            callback_group = cb_Navigation
        )
        self._range_subscription = self.create_subscription (
            Range,
            f'/{roomba_name}/ir',
            self.ir_callback,
            qos_profile_sensor_data,
            callback_group=cb_Navigation
        )
        self._cobot_subscription = self.create_subscription (
            String,
            f'/{cobot_name}/message_cobot',
            self.listener_callback,
            qos_profile_system_default,
            callback_group = cb_Communication
        )
    
    def odometer_callback(self, msg):
        self._current_pose = msg.pose.pose #pose.pose is pose, contains position
    def ir_callback(self, msg):
        self.get_logger().warning('nice')
        self.get_logger().warning(str(msg.range))
    def listener_callback(self, msg):
        if msg.data == "Box placed":
            self.execute_walk()
        if msg.data == "Box taken":
            self.get_logger().warning('FINISHED')
    
    def execute_start(self):
        self.do_undock()
        self.do_drive(0.2)
        self._dock_position = self._current_pose.position
        self.do_turn(-1.57)
        self.do_drive(1.0)
        self.execute_pickup()
    def execute_pickup(self):
        self.do_turn(3.14)
        self.do_drive(1.0)
        self.do_turn(1.57)
        self.do_dock()
        message = String()
        message.data = "Place box"
        self._publisher.publish(message)
        # time.sleep(3)
        # self.execute_walk()
    def execute_walk(self):
        self.do_undock()
        self.do_drive(1.5)
        self.do_turn(-1.0)
        self.do_drive(1.0)
        self.do_turn(3.5)
        self.do_drive(1.0)
        self.do_turn(10.0)
        self.do_drive(0.5)
        self.execute_return()
        message = String()
        message.data = "Remove box"
        self._publisher.publish(message)
        # time.sleep(3)
        # self.get_logger().warning('FINISHED')
    def execute_return(self):
        self.do_turn(self.calculate_turnangle(self._current_pose, self._dock_position))
        self.do_drive(self.calculate_distance(self._current_pose.position, self._dock_position))
        self.do_dock()
    
    def do_drive(self, distance):
        self.get_logger().warning('DRIVE ' + str(distance))
        action_client = self._drive_action_client
        action_client.wait_for_server()
        goal = DriveDistance.Goal()
        goal.distance = distance
        with lock:
            handle = action_client.send_goal_async(goal)
            while not handle.done():
                pass
            self._goal_uuid = handle.result()
        while self._goal_uuid.status == GoalStatus.STATUS_UNKNOWN:
            pass
        while self._goal_uuid.status is not GoalStatus.STATUS_CANCELED:
            if self._goal_uuid.status is GoalStatus.STATUS_SUCCEEDED:
                break
            pass
        with lock:
            self.get_logger().warning('cancelled')
            self._goal_uuid = None

    def do_turn(self, angle):
        self.get_logger().warning('TURN ' + str(angle))
        action_client = self._rotate_action_client
        action_client.wait_for_server()
        goal = RotateAngle.Goal()
        goal.angle = angle
        action_client.send_goal(goal)
    def do_undock(self):
        self.get_logger().warning('UNDOCK')
        action_client = self._undock_action_client
        action_client.wait_for_server()
        goal = Undock.Goal()
        action_client.send_goal(goal)
    def do_dock(self):
        self.get_logger().warning('DOCK')
        action_client = self._dock_action_client
        action_client.wait_for_server()
        goal = Dock.Goal()
        action_client.send_goal(goal)
        self.get_logger().warning('DOCKED!')
    
    """def calculate_turnangle(self, current_pose, target_position):
        current_yaw = euler_from_quaternion([
            current_pose.orientation.x, 
            current_pose.orientation.y, 
            current_pose.orientation.z, 
            current_pose.orientation.w
        ])[2]
        delta_x = target_position.x - current_pose.position.x
        delta_y = target_position.y - current_pose.position.y
        return math.atan2(delta_y, delta_x) - current_yaw"""
    def calculate_distance(self, current_position, target_position):
        delta_x = target_position.x - current_position.x
        delta_y = target_position.y - current_position.y
        return math.sqrt(delta_x**2 + delta_y**2)


def main(args = None):
    rclpy.init()
    roomba_driver = RoombaDriver()
    executor = MultiThreadedExecutor(2)
    executor.add_node(roomba_driver)
    keycom = KeyCommander ([
        (KeyCode(char='r'), roomba_driver.execute_start),
    ])
    print("r: Start")
    try:
        executor.spin()
    except KeyboardInterrupt:
        print('KeyboardInterrupt, shutting down.')
        print("Shutting down executor")
        executor.shutdown()
        print("Destroying Node")
        roomba_driver.destroy_node()
        print("Shutting down RCLPY")
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
