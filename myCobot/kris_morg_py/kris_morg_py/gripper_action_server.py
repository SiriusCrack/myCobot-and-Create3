#!/usr/bin/env python3
import rclpy
from pymycobot.mycobot import MyCobot

from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from kris_morg_interfaces.action import Gripper

cobot_name = "velma"

class GripServer(Node):
    def __init__(self):
        super().__init__('grip_server',namespace=cobot_name)

        self.goal = Gripper.Goal()
        self.mc = MyCobot("/dev/ttyAMA0", 115200)

        self._action_server = ActionServer(self, Gripper, f'/{cobot_name}/gripper', 
                                           execute_callback = self.execute_callback,
                                           goal_callback = self.goal_callback)
        
    def goal_callback(self, goal_request):
        # Accepts or Rejects client request to begin action
        self.goal = goal_request
        return GoalResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        state = self.goal.state #int 1 or 0
        speed = self.goal.speed #default 80
    
        if state > 1 or state < 0:
            raise ValueError("state must be either a 1 or 0")

        GoalResponse.EXECUTING 
        
        print('State:', "Close" if state == 1 else "Open")
        print('Speed:',speed)

        # Do goal
        self.mc.set_gripper_state(state, speed)

        while True:
            #print("Moving!")
            if not self.mc.is_gripper_moving(): # If its done moving
                # Set everything to completed
                goal_handle.succeed()
                result = Gripper.Result()
                result.success = True
                GoalResponse.SUCCEEDED
                return result

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init()

    grip_action_server = GripServer()

    rclpy.spin(grip_action_server)

    grip_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
