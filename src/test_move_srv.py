#!/usr/bin/env python

import sys
import rospy
from tf.transformations import quaternion_from_euler
from romina2.srv import *

def move(trans, rot):
    rospy.wait_for_service('/pilot/move')
    try:
        move = rospy.ServiceProxy('/pilot/move', Move)
        req = MoveRequest()
        req.movement.translation.x = trans
        q = quaternion_from_euler(0,0,rot);
        req.movement.rotation.x = q[0]
        req.movement.rotation.y = q[1]
        req.movement.rotation.z = q[2]
        req.movement.rotation.w = q[3]
        print move(req)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    move(1, 0)
