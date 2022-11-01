#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Float64

def talker():
    pub = rospy.Publisher('chatter', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        pub.publish(1)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass