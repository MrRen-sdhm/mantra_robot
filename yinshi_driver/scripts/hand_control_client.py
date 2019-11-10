#!/usr/bin/env python

import rospy
from yinshi_driver.srv import *


def hand_contol_client(command):
    rospy.wait_for_service('hand_control')
    try:
        hand_control = rospy.ServiceProxy('hand_control', HandControl)
        resp = hand_control(command)
        return resp.success
    except rospy.ServiceException, e:
        print "[SRVICE] Service call failed: %s" % e


def usage():
    return "%s [command] Supported command: Open/Close" % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 2:
        command = sys.argv[1]
    else:
        print usage()
        sys.exit(1)

    if (command != "Open") and (command != "Close"):
        print "[ERROR] Supported command: Open/Close"
    else:
        print "[SRVICE] Requesting %s" % command
        print "[SRVICE] Result = %s" % hand_contol_client(command)

