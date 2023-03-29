#!/usr/bin/env python3
import rclpy
from pymycobot.mycobot import MyCobot

from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from kris_morg_interfaces.action import Move

class MoveServer(Node):
    def __init__(self):
        super().__init__('move_server')

        self.goal = Move.Goal()
        self.mc = MyCobot("/dev/ttyAMA0", 115200)

        self._action_server = ActionServer(self, Move, 'move', 
                                           execute_callback = self.execute_callback,
                                           goal_callback = self.goal_callback)
        
    def goal_callback(self, goal_request):
        # Accepts or Rejects client request to begin action
        self.goal = goal_request
        return GoalResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        joints = self.goal.joints #float list[6]
        speed = self.goal.speed #default 80, also, how work?

        print(joints)
        print(speed)

        #self.mc.send_radians(joints,speed)
        result = Move.Result()
        result.success = True
        return result

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init()

    move_action_server = MoveServer()

    rclpy.spin(move_action_server)

    move_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()