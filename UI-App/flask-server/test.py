#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub =rospy.Publisher('TargetLocation', String, queue_size=10)
    rospy.init_node('Sender', anonymous=True)
    rate = rospy.Rate(10) #10 hz
    while not rospy.is_shutdown():
        targetLoc = "Target Location changed at %s" % rospy.get_time()
        rospy.loginfo(targetLoc)
        pub.publish(targetLoc)
        rate.sleep()
    
    if name == 'main':
        try:
            Sender()
        except rospy.ROSInterruptException:
            pass