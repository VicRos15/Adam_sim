#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import scipy.io


class Node():

    def __init__(self, mode):
        #Define publishers
        self.pub_right_joints=None
        self.pub_left_joints=None

        #Define joint data for subscribers
        self.right_joints=[]
        self.left_joints=[]
                
        # Mode == 1, publisher
        # Mode == 0, listener

        rospy.init_node('connect_to_robot_node', anonymous=True)

        if mode == 1:
            #Create publishers
            self.pub_right_joints = rospy.Publisher('/right_joints', Float64MultiArray, queue_size=10)
            self.pub_left_joints = rospy.Publisher('/left_joints', Float64MultiArray, queue_size=10)

        elif mode == 0:
            #Create subscribers
            rospy.Subscriber('/right_joints', Float64MultiArray, self.callback_right_joints)
            rospy.Subscriber('/left_joints', Float64MultiArray, self.callback_left_joints)
        else:
            raise ValueError("Mode debe ser 0 o 1")

    def publish_joints(self, pub_arm, joints_value):
        if pub_arm == "right":
            right_joint_msg = Float64MultiArray()
            right_joint_msg.data = joints_value
            self.pub_right_joints.publish(right_joint_msg)
            rospy.loginfo(f"Published right joints: {self.right_joints}")
        if pub_arm == "left":
            left_joint_msg = Float64MultiArray()
            left_joint_msg.data = joints_value
            self.pub_left_joints.publish(left_joint_msg)
            rospy.loginfo(f"Published left joints: {self.left_joints}")


    def callback_right_joints(self, msg):
        self.right_joints = msg.data
        rospy.loginfo(f"Right joints updated: {self.right_joints}")



    def callback_left_joints(self,msg):
        self.left_joints = msg.data
        rospy.loginfo(f"Left joints updated: {self.left_joints}")

    
    def read_joints(self):

        rospy.spin()







