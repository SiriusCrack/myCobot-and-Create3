#!/usr/bin/env python3
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
            # Open grip
            self.grip(0) 
            # Move to box
            self.toBox()
            # Close grip, grabbing box
            self.grip(1)
            # Move to the Roomba
            self.toRoomba()
            # Let go of the box
            self.grip(0)

            #publish 'done' to Roomba
            self._publisher_.publish('Done')
            return
        elif msg == 'Remove box':
            # Open grip
            self.grip(0)
            # Move to the Roomba
            self.toRoomba()
            # Close grip, grabbing box
            self.grip(1)
            # Move to box
            self.toBox()
            # Let go of the box
            self.grip(0)

            #publish 'done' to Roomba
            self._publisher_.publish('Done')
            return
        else:
            #If here, there was an issue....?
            return

    def toBox(self):
        """
        This function will move the arm to the box location for either placing or
        grabbing.
        If its picking up the box, remember to open gripper first! Otherwise 
        you will push the box away...
        """
        action_client = self.move_action_client
        action_client.wait_for_server()
        goal = Move.Goal()
        goal.joints = [] #List of radians to move
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
        If its picking up the box, remember to open gripper first! Otherwise 
        you will push the box away...
        """
        action_client = self.move_action_client
        action_client.wait_for_server()
        goal = Move.Goal()
        goal.joints = [] #List of radians to move
        action_client.send_goal(goal)


def main(args=None):
    rclpy.init()

    roombaTime = DoteOnRoomba()

    rclpy.spin(roombaTime)

    roombaTime.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

