#!/usr/bin/env python3
"""
Before running this, make sure that the move and gripper action servers are 
running! 
To run:
ros2 run kris_morg_py with_roomba_exe
"""

import rclpy
from pymycobot.mycobot import MyCobot
from rclpy.action import ActionClient, GoalResponse
from rclpy.node import Node
from kris_morg_interfaces.action import Move, Gripper
from std_msgs.msg import String
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time

cobot_name = "velma"
roomba_name = "create3_03EE"

class DoteOnRoomba(Node):
    def __init__(self):
        super().__init__('dote_on_roomba')

        cb_Action = MutuallyExclusiveCallbackGroup()
        cb_Communication = MutuallyExclusiveCallbackGroup()

        self.move_action_client = ActionClient(self, Move, f'/{cobot_name}/move', callback_group = cb_Action)
        self.grip_action_client = ActionClient(self, Gripper, f'/{cobot_name}/gripper', callback_group = cb_Action)
        # Subscription from Roomba (Roomba -> MyCobot)
        self._subscription_ = self.create_subscription(
            String, f'/{roomba_name}/message_roomba', 
            self.listener_callback, 
            10,
            callback_group = cb_Communication) # Queue size
        
        # Publisher to Roomba (MyCobot -> Roomba)
        self._publisher_ = self.create_publisher(String, # Type of message
                                                f'/{cobot_name}/message_cobot', 
                                                10) # 10 = Queue Size
        # Variables
        self._roomba_ready = False
    def listener_callback(self, msg):
        '''
        This function is called every time self.subscription gets a message
        from the Roomba. Here it parses the message from the Roomba and determines
        which action it will need to take.
        '''
        if msg.data == "Place box":
            # Move to box
            self.toBox()
            time.sleep(1)
            # Open grip
            self.grip(0) 
            time.sleep(1)
            # Lower to box
            self.lowerToGrab(0)
            time.sleep(1)
            # Close grip, grabbing box
            self.grip(1)
            time.sleep(1)
            # Reset arm
            self.resetPos()
            time.sleep(1)
            # Move to the Roomba
            self.toRoomba()
            time.sleep(1)
            # Lower to box placement
            self.lowerToGrab(1)
            time.sleep(1)
            # Let go of the box
            self.grip(0)
            time.sleep(1)
             # Reset arm
            self.resetPos()
            # Publish 'done' to Roomba
            message = String()
            message.data = "Box placed"
            self._publisher_.publish(message)
            return
        elif msg.data == "Remove box":
            # Move to the Roomba
            self.toRoomba()
            time.sleep(1)
            # Open grip
            self.grip(0)
            time.sleep(1)
            # Lower to box
            self.lowerToGrab(1)
            time.sleep(1)
            # Close grip, grabbing box
            self.grip(1)
            time.sleep(1)
            # Reset arm
            self.resetPos()
            time.sleep(1)
            # Move to box
            self.toBox()
            time.sleep(1)
            # Lower to box placement
            self.lowerToGrab(0)
            time.sleep(1)
            # Let go of the box
            self.grip(0)
            time.sleep(1)
            # Reset arm
            self.resetPos()
            # Publish 'done' to Roomba
            message = String()
            message.data = "Box taken"
            self._publisher_.publish(message)
            
            return
        else:
            #If here, there was an issue....?
            print("Ooops. I'm in the else...")
            return

    def toBox(self):
        """
        This function will move the arm to the box location for either placing or
        grabbing.
        """
        action_client = self.move_action_client
        action_client.wait_for_server()
        goal = Move.Goal()
        goal.joints = [0.0,1.0,1.0,1.0,-1.2,0.0] # Just above the cube
        action_client.send_goal(goal)
    def grip(self, state):
        """
        Open/close the gripper via ROS. 0 is open, 1 is close
        """
        action_client = self.grip_action_client
        action_client.wait_for_server()
        goal = Gripper.Goal()
        goal.state = state 
        action_client.send_goal(goal)
    def toRoomba(self):
        """
        This function will move the arm to the Roomba for either placing or
        grabbing.
        """
        action_client = self.move_action_client
        action_client.wait_for_server()
        goal = Move.Goal()
        goal.joints = [2.8,0.7,0.5,0.0,-1.5,0.0] # Just above the cube
        action_client.send_goal(goal)
    def lowerToGrab(self, where):
        """
        Precise slow movement to grab/place box
        """
        if where == 0: # Inital box State
            action_client = self.move_action_client
            action_client.wait_for_server()
            goal = Move.Goal()
            goal.joints = [0.0,1.4,0.7,1.0,-1.2,0.0] # Lower to the cube
            goal.speed = 30
            action_client.send_goal(goal)
        elif where == 1: # Over the Roomba
            action_client = self.move_action_client
            action_client.wait_for_server()
            goal = Move.Goal()
            goal.joints = [2.8,0.7,0.6,0.0,-1.5,0.0] # Lower to the cube
            goal.speed = 30
            action_client.send_goal(goal)
    def resetPos(self):
        action_client = self.move_action_client
        action_client.wait_for_server()
        goal = Move.Goal()
        goal.joints = [0.0,0.0,0.0,0.0,0.0,0.0] 
        action_client.send_goal(goal)
        self.grip(1)

    def destroy(self):
        super().destroy_node()

def main(args=None):
    try:
        rclpy.init()
        roombaTime = DoteOnRoomba()
        executor = MultiThreadedExecutor(2)
        executor.add_node(roombaTime)
        executor.spin()
        #while True: # Keep running until theres a forced stop (just in case)
        #rclpy.spin(roombaTime)
    except KeyboardInterrupt: # Wait for manual shutdown
        roombaTime.destroy()
        rclpy.shutdown()
if __name__ == '__main__':
    main()