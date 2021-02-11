#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import random
class MiraJointMover():
    def __init__(self):
        rospy.init_node('jointmover_demo')
        self.pub_mira_roll_joint_position = rospy.Publisher('/mira/roll_joint_position_controller/command', Float64,queue_size=1)
        self.pub_mira_pitch_joint_position = rospy.Publisher('/mira/pitch_joint_position_controller/command',Float64,queue_size=1)
        self.pub_mira_yaw_joint_position = rospy.Publisher('/mira/yaw_joint_position_controller/command',Float64,queue_size=1) 
        rospy.Subscriber("/mira/joint_states", JointState, self.mira_joints_callback)
        self.roll_angle = Float64()
        self.pitch_angle = Float64()
        self.yaw_angle = Float64()

    def move_mira_all(self, roll, pitch, yaw):
        self.roll_angle.data = roll
        self.pitch_angle.data = pitch
        self.yaw_angle.data = yaw
        self.pub_mira_roll_joint_position.publish(self.roll_angle)
        self.pub_mira_pitch_joint_position.publish(self.pitch_angle)
        self.pub_mira_yaw_joint_position.publish(self.yaw_angle)

    def mira_joints_callback(self, msg):
        self.move_mira = msg
        return self.move_mira
        

    def movement_random_loop(self):
        while not rospy.is_shutdown():
            roll = random.uniform(-0.15, 0.15)
            pitch = random.uniform(0.0, 0.3)
            yaw = random.uniform(0.0, 2*5)
            self.move_mira_all(roll,pitch,yaw)
    

if __name__ == "__main__":
    mira_jointmover_object = MiraJointMover()
    mira_jointmover_object.movement_random_loop()