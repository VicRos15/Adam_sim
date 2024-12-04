#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import scipy.io


class Node():

    def __init__(self):

        self.left_topic = '/robot/left_arm/joint_states'
        self.right_topic = '/robot/right_arm/joint_states'

        #Data buffers
        self.left_data= {
            "name": [],
            "position": [],
            "velocity": [],
            "effort": []
        }

        self.right_data= {
            "name": [],
            "position": [],
            "velocity": [],
            "effort": []
        }

        #Define publishers
        self.pub_right_joints=None
        self.pub_left_joints=None

        #Define joint data for subscribers
        self.right_joints=[]
        self.left_joints=[]
                
        # Mode == 1, publisher
        # Mode == 0, listener
        rospy.init_node('connect_to_robot_node', anonymous=True)


    def publish_joints(self, pub_arm, joints_value):
        if pub_arm == "right":
            right_joint_msg = Float64MultiArray()
            right_joint_msg.data = joints_value
            self.pub_right_joints.publish(right_joint_msg)
            # rospy.loginfo(f"Published right joints: {self.right_joints}")
        if pub_arm == "left":
            left_joint_msg = Float64MultiArray()
            left_joint_msg.data = joints_value
            self.pub_left_joints.publish(left_joint_msg)
            # rospy.loginfo(f"Published left joints: {self.left_joints}")

    #getters
    def get_right(self):
        return self.right_data
    
    def get_left(self):
        return self.left_data


    def callback_read_left(self, msg):

        # self.left_data["name"]= msg.name
        # self.left_data["position"]=msg.position
        # self.left_data["velocity"]=msg.velocity
        # self.left_data["effort"]=msg.effort

        rospy.set_param('name_left',msg.name)
        rospy.set_param('position_left',msg.position)
        rospy.set_param('velocity_left',msg.velocity)
        rospy.set_param('effort_left',msg.effort)

        # print(self.left_data)

        # rospy.loginfo(f"Right joints updated: {self.right_joints}")

    def callback_read_right(self, msg):
        
        # self.right_data["name"] = msg.name
        # self.right_data["position"]=msg.position
        # self.right_data["velocity"]=msg.velocity
        # self.right_data["effort"]=msg.effort


        rospy.set_param('name_right',msg.name)
        rospy.set_param('position_right',msg.position)
        rospy.set_param('velocity_right',msg.velocity)
        rospy.set_param('effort_right',msg.effort)
        
        # print(self.right_data)



    def read_joints(self,arm):

        if arm == "right":
            rospy.Subscriber(self.right_topic, JointState, self.callback_read_right)

        elif arm == "left":
            rospy.Subscriber(self.left_topic, JointState, self.callback_read_left)

        elif arm == "both":
            rospy.Subscriber(self.left_topic, JointState, self.callback_read_left)
            rospy.Subscriber(self.right_topic, JointState, self.callback_read_right)

        else:
            raise ValueError("Mode debe ser 0 o 1")
        
        # rospy.spin()
        


        

if __name__=='__main__':
    node = Node()

    node.read_joints("right")







