#!/usr/bin/env python

""" 
@package cluedo user interface
This node displays to the user
some messages from the game
"""
import time
import rospy
from std_msgs.msg import String

def callback(msg):
    """
    /brief it is the callback of the user_interface topic 
    it receives a message of type String and and displays it
    on the screen
    """
    print ('______________________________________________________________________________')
    print (msg.data)
    print ('______________________________________________________________________________')
    
def user_interface():
    """
    /brief it is the subscriber of the user_interface topic 
    it receives a string and the callback displays the message
    in the bash
    """
    
    rospy.init_node('user_interface', anonymous=True)

    rospy.Subscriber("cluedo_ui", String, callback)

    while not rospy.is_shutdown():
       rospy.spin()

if __name__ == '__main__':
   user_interface()
   while not rospy.is_shutdown():
       rospy.spin()
