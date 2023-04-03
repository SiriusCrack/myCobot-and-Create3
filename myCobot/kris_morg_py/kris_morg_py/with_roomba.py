#!/usr/bin/env python3
import rclpy
from pymycobot.mycobot import MyCobot
from rclpy.action import ActionClient, GoalResponse
from rclpy.node import Node
from kris_morg_interfaces.action import Move
from std_msgs.msg import String

cobot_name = "velma"
roomba_name = None

class DoteOnRoomba(Node):
    def __init__(self):
        super().__init__('dote_on_roomba')

        # 2 Seperate Callback Groups for handling the Roomba Subscription and Action Client
        #cb_Subscripion = MutuallyExclusiveCallbackGroup()
        #cb_Action = MutuallyExclusiveCallbackGroup()

        self._action_client = ActionClient(self, Move, f'/{cobot_name}/move')#, callback_group = cb_Action)
        # Subscription from Roomba (Roomba -> MyCobot)
        #self._subscription_ = self.create_subscription(
        #    String, f'/{roomba_name}/<roombaNode>', 
        #    self.listener_callback, Queue = X,
        #    callback_group=cb_Subscripion)
        # Publisher to Roomba (MyCobot -> Roomba)
        self._publisher_ = self.create_publisher(String, # Type of message
                                                f'/{cobot_name}/message_roomba', 
                                                10) # 10 = Queue Size
        """
        Subcription to publisher above:
        self.subscription = self.create_subscription(
            String,
            f'/{cobot_name}/message_roomba',
            self.listener_callback,
            10)
        """
        # Variables
        self._roomba_ready = False

    def listener_callback(self, msg):
        '''
        This function is called every time self.subscription gets a message
        from the Roomba. Here it parses the message from the Roomba and determines
        which action it will need to take.
        '''
        if msg == 'Place box':
            #place box
            #publish 'done' to Roomba
            self._publisher_.publish('Done')
            pass
        elif msg == 'Remove box':
            #remove box
            #publish 'done' to Roomba
            self._publisher_.publish('Done')
            pass
        else:
            #If here, there was an issue....?
            pass

def main(args=None):
    rclpy.init()

    roombaTime = DoteOnRoomba()

    rclpy.spin(roombaTime)

    roombaTime.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
