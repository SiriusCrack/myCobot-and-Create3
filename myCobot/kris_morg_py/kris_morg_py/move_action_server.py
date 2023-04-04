#!/usr/bin/env python3
import rclpy
from pymycobot.mycobot import MyCobot

from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from kris_morg_interfaces.action import Move

cobot_name = "velma"

class MoveServer(Node):
    def __init__(self):
        super().__init__('move_server',namespace=cobot_name)

        self.goal = Move.Goal()
        self.mc = MyCobot("/dev/ttyAMA0", 115200)

        self._action_server = ActionServer(self, Move, f'/{cobot_name}/move', 
                                           execute_callback = self.execute_callback,
                                           goal_callback = self.goal_callback)
        
    def goal_callback(self, goal_request):
        # Accepts or Rejects client request to begin action
        self.goal = goal_request
        return GoalResponse.ACCEPT
    
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

    move_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
