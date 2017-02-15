#!/usr/bin/env python

import time
import rospy
from std_srvs.srv import Empty

ns = '/choreonoid/'

def pause():
    rospy.wait_for_service(ns + 'pause_physics')
    try:
        pausesrv = rospy.ServiceProxy(ns + 'pause_physics', Empty)
        pausesrv()
    except rospy.ServiceException, e:
        print "Service call failed %s" % e

def unpause():
    rospy.wait_for_service(ns + 'unpause_physics')
    try:
        unpausesrv = rospy.ServiceProxy(ns + 'unpause_physics', Empty)
        unpausesrv()
    except rospy.ServiceException, e:
        print "Service call failed %s" % e

while True:
    print "pause"
    pause()
    time.sleep(5)
    print "unpause"
    unpause()
    time.sleep(5)
