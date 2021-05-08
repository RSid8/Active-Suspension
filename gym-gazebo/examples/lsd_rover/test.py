#!/usr/bin/env python3

import numpy as np
import torch as T
import torch.nn.functional as F
import torch.nn as nn
import torch.optim as optim
import os
import matplotlib.pyplot as plt 
import gym
import gym_gazebo
import time
import rospy
from std_msgs.msg import Float64
from math import radians
import pandas as pd
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
def callback_imu(msg):
    global myvar
    orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z,
                                 msg.orientation.w]
    (roll,pitch,yaw)=euler_from_quaternion(orientation_list)
    myvar=pitch

def callback_pose(msg):
    global x
    x = msg.pose.pose.position.x

def forward(velocity_publisher):
    vel_cmd = Twist()
    vel_cmd.linear.x = -0.49
    vel_cmd.linear.y = 0
    vel_cmd.angular.z =0
    velocity_publisher.publish(vel_cmd)

if __name__ == '__main__':
    
    env = gym.make('GazeboMarsLsdForce-Lidar-v0')
    rospy.Subscriber("/imu", Imu, callback_imu)
    rospy.Subscriber("/odom", Odometry, callback_pose)
    velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    joint_1_publisher = rospy.Publisher("/lsd/fl_joint_position_controller/command",
                                                 Float64, queue_size=10)
    joint_2_publisher = rospy.Publisher("/lsd/fl_joint_position_controller/command",
                                                 Float64, queue_size=10)
    joint_3_publisher = rospy.Publisher("/lsd/fl_joint_position_controller/command",
                                                 Float64, queue_size=10)
    joint_4_publisher = rospy.Publisher("/lsd/fl_joint_position_controller/command",
                                                 Float64, queue_size=10)
    time.sleep(15)
    done=False
    myvar=None
    x=None
    #fl,fr,bl,br
    a=np.array([0,0,0,0])
    keys=['pitch', 'x_displacement']
    new_dict = dict(zip(keys, ([] for _ in keys)))
    x=0
    while x<8:
        #env.step(a)
        #flag=1
        #obs = env.get_observation()
        forward(velocity_publisher)
        if(2.9<x<3):
            a[2]+=30
            a[3]+=30

            joint_1_publisher.publish(radians(a[0]))
            joint_2_publisher.publish(radians(a[1]))
            joint_3_publisher.publish(radians(a[2]))
            joint_4_publisher.publish(radians(a[3]))

            time.sleep(0.5)

            time.sleep(4)

            a[2]=-a[2]
            a[3]=-a[3]

            joint_1_publisher.publish(radians(a[0]))
            joint_2_publisher.publish(radians(a[1]))
            joint_3_publisher.publish(radians(a[2]))
            joint_4_publisher.publish(radians(a[3]))

            time.sleep(0.5)

            time.sleep(5)

            joint_1_publisher.publish(radians(10))
            joint_2_publisher.publish(radians(10))
            joint_3_publisher.publish(radians(0))
            joint_4_publisher.publish(radians(0))

            time.sleep(2)

            joint_1_publisher.publish(radians(0))
            joint_2_publisher.publish(radians(0))
            joint_3_publisher.publish(radians(0))
            joint_4_publisher.publish(radians(0))

            time.sleep(1)
        new_dict['pitch'].append(myvar)
        new_dict['x_displacement'].append(x)

    df=pd.DataFrame(new_dict)
    df.to_csv('values.csv', index=True)
     


        

    
    
        
    

    


        
   
        
