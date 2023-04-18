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

cobot_name = "velma"
roomba_name = None

class DoteOnRoomba(Node):
    def __init__(self):
        super().__init__('dote_on_roomba')
        self.move_action_client = ActionClient(self, Move, f'/{cobot_name}/move')
        self.grip_action_client = ActionClient(self, Gripper, f'/{cobot_name}/gripper')
        # Subscription from Roomba (Roomba -> MyCobot)
        self._subscription_ = self.create_subscription(
            String, f'/{roomba_name}/message_roomba', 
            self.listener_callback, 
            10) # Queue size
        
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
        if msg == 'Place box':
            # Move to box
            self.toBox()
            # Open grip
            self.grip(0) 
            # Lower to box
            self.lowerToGrab(0)
            # Close grip, grabbing box
            self.grip(1)
            # Move to the Roomba
            self.toRoomba()
            # Lower to box placement
            self.lowerToGrab(1)
            # Let go of the box
            self.grip(0)
            # Publish 'done' to Roomba
            self._publisher_.publish('Done')
            # Reset arm
            self.resetPos()
            return
        elif msg == 'Remove box':
            # Move to the Roomba
            self.toRoomba()
            # Open grip
            self.grip(0)
            # Lower to box
            self.lowerToGrab(1)
            # Close grip, grabbing box
            self.grip(1)
            # Move to box
            self.toBox()
            # Lower to box placement
            self.lowerToGrab(0)
            # Let go of the box
            self.grip(0)
            #publish 'done' to Roomba
            self._publisher_.publish('Done')
            # Reset arm
            self.resetPos()
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
        goal.joints = [0,1,1,1,-1.2,0] # Just above the cube
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
        goal.joints = [] # Just above the cube
        action_client.send_goal(goal)
    def lowerToGrab(self, where):
        """
        Precise slow movement to grab/place box
        """
        if where == 0: # Inital State
            action_client = self.move_action_client
            action_client.wait_for_server()
            goal = Move.Goal()
            goal.joints = [0,1.4,0.7,1,-1.2,0] # Lower to the cube
            goal.speed = 30
            action_client.send_goal(goal)
        elif where == 1: # Over the Roomba
            action_client = self.move_action_client
            action_client.wait_for_server()
            goal = Move.Goal()
            goal.joints = [] # Lower to the cube
            goal.speed = 30
            action_client.send_goal(goal)
    def resetPos(self):
        action_client = self.move_action_client
        action_client.wait_for_server()
        goal = Move.Goal()
        goal.joints = [0,0,0,0,0,0] 
        action_client.send_goal(goal)

def main(args=None):
    try:
        rclpy.init()
        roombaTime = DoteOnRoomba()
        #while True: # Keep running until theres a forced stop (just in case)
        rclpy.spin(roombaTime)
    except KeyboardInterrupt: # Wait for manual shutdown
        roombaTime.destroy()
        rclpy.shutdown()
if __name__ == '__main__':
    main()