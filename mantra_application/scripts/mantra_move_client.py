#!/usr/bin/env python

import rospy
from mantra_application.srv import *


def mantra_move_client(pose_name):
    rospy.wait_for_service('move_to_pose_named')
    try:
        move_to_pose_named = rospy.ServiceProxy('move_to_pose_named', MoveToPoseNamed)
        resp = move_to_pose_named(pose_name)
        return resp.success
    except rospy.ServiceException, e:
        print "[SRVICE] Service call failed: %s" % e


def usage():
    return "%s [pose_name]" % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 2:
        pose_name = sys.argv[1]
    else:
        print usage()
        sys.exit(1)
    print "[SRVICE] Requesting %s" % pose_name
    print "[SRVICE] Move result1 = %s" % mantra_move_client(pose_name)
    print "[SRVICE] Move result2 = %s" % mantra_move_client("home")
