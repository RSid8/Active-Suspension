#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from math import radians




def calculate():
    angle = radians(-30)
    pub1.publish(angle)
    pub2.publish(angle)
    pub3.publish(angle)
    pub4.publish(angle)

def listener():

    while not rospy.is_shutdown():
        calculate()
        rospy.sleep(0.01)

if __name__ == '__main__':
    try:

        pub1 = rospy.Publisher('/lsd/joint1_position_controller/command', Float64, queue_size=10)
        pub2 = rospy.Publisher('/lsd/joint2_position_controller/command', Float64, queue_size=10)
        pub3 = rospy.Publisher('/lsd/joint3_position_controller/command', Float64, queue_size=10)
        pub4 = rospy.Publisher('/lsd/joint4_position_controller/command', Float64, queue_size=10)
        rospy.init_node('Communication', anonymous=True, disable_signals=True)
        rate = rospy.Rate(50)

        listener()

    except rospy.ROSInterruptException:
        pass
