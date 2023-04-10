#!/usr/bin/env python3
"""
To run this:
ros2 run kris_morg_py move_action_server_exe
"""
import rclpy
from pymycobot.mycobot import MyCobot

from rclpy.action import ActionServer, GoalResponse
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from kris_morg_interfaces.action import Move
from rclpy.executors import MultiThreadedExecutor

cobot_name = "velma"

class MoveServer(Node):
    def __init__(self):
        super().__init__('move_server',namespace=cobot_name)
        self.goal = Move.Goal()
        self.mc = MyCobot("/dev/ttyAMA0", 115200)

        self._action_server = ActionServer(self, Move, f'/{cobot_name}/move', 
                                           execute_callback = self.execute_callback,
                                           goal_callback = self.goal_callback,
                                           cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        # Accepts or Rejects client request to begin action
        self.goal = goal_request
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        #self.get_logger().info('Received cancel request :(')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        joints = self.goal.joints #float list[6]
        speed = self.goal.speed #default 80, also, how work?

        GoalResponse.EXECUTING 
        

        print('Joint radians:',joints)
        print('Speed:',speed)

        # Do goal
        self.mc.send_radians(joints,speed)

        while True:
            #print("Moving!")
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                #self.get_logger().info('Goal canceled')
                GoalResponse.CANCELED
                return Move.Result()

            if(self.mc.is_moving()): # If its done moving
                # Set everything to completed
                goal_handle.succeed()
                result = Move.Result()
                result.success = True
                GoalResponse.SUCCEEDED
                return result
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()
def main(args=None):
    rclpy.init()

    move_action_server = MoveServer()

    rclpy.spin(move_action_server)
    rclpy.spin(move_action_server, executor = MultiThreadedExecutor())

    move_action_server.destroy()
    rclpy.shutdown()
if __name__ == '__main__':
    main()