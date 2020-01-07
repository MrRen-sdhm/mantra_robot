#!/usr/bin/env python

import rospy
from yinshi_driver.srv import *


def hand_control_client(command, pose):
    if pose is None:
        pose = -1
    else:
        pose = int(pose)

    rospy.wait_for_service('hand_control')
    try:
        hand_control = rospy.ServiceProxy('hand_control', HandControl)
        resp = hand_control(command, pose)
        return resp.success
    except rospy.ServiceException, e:
        print "[SRVICE] Service call failed: %s" % e


def usage():
    return "%s [command] Supported command: Open/Close/Move" % sys.argv[0]


if __name__ == "__main__":
    pose = None
    if len(sys.argv) == 2:
        command = sys.argv[1]
    elif len(sys.argv) == 3:
        command = sys.argv[1]
        pose = sys.argv[2]
    else:
        print usage()
        sys.exit(1)

    if (command != "Open") and (command != "Close") and (command != "Move"):
        print "[ERROR] Supported command: Open/Close/Move"
    elif command == "Move" and pose == None:
        print "[ERROR] Please specify the pose [1-1000]"
    else:
        print "[SRVICE] Requesting %s" % command
        print "[SRVICE] Result = %s" % hand_control_client(command, pose)

