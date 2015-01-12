#!/usr/bin/env python  
import rospy

import tf
import turtlesim.msg

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')

    br = tf.TransformBroadcaster()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
    	br.sendTransform((.1,0,.1),
                     tf.transformations.quaternion_from_euler(0, 1.57, 0),
                     rospy.Time.now(),
                     "usb_cam",
                     "robot")
	r.sleep()


    rospy.spin()
