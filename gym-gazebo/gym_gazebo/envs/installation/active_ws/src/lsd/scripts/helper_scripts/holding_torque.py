#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Wrench


ob1 = Wrench()
ob2 = Wrench()
ob3 = Wrench()
ob4 = Wrench()



def calculate():
    global ob1,ob2,ob3,ob4

    ob1.torque.y = -4.8
    ob2.torque.y = -4.8
    ob3.torque.y = -6.6
    ob4.torque.y = -6.6

    pub1.publish(ob1)
    pub2.publish(ob2)
    pub3.publish(ob3)
    pub4.publish(ob4)


def listener():


    while not rospy.is_shutdown():
        calculate()
        rospy.sleep(0.01)

if __name__ == '__main__':
    try:

        pub1 = rospy.Publisher('fl_motor_ft_apply', Wrench, queue_size=10)
        pub2 = rospy.Publisher('fr_motor_ft_apply', Wrench, queue_size=10)
        pub3 = rospy.Publisher('bl_motor_ft_apply', Wrench, queue_size=10)
        pub4 = rospy.Publisher('br_motor_ft_apply', Wrench, queue_size=10)
        rospy.init_node('holding_torque_node', anonymous=True, disable_signals=True)
        rate = rospy.Rate(50)

        listener()

    except rospy.ROSInterruptException:
        pass
