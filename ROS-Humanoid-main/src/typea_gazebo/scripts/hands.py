#!/usr/bin/env python3

import rospy
from typea_gazebo.typea import TypeA
from geometry_msgs.msg import Twist

rospy.init_node("Hands")
rospy.sleep(1)

rospy.loginfo("TypeA connecting...")
typea = TypeA()



angle = {
    "l_shoulder_lateral_joint" : 1.5708,
    "r_shoulder_lateral_joint" : 1.5708,
    "l_shoulder_swing_joint" : 1.5708,
    "r_shoulder_swing_joint" : -1.5708
    }

typea.set_angles(angle)
p = typea.get_angles()

rospy.loginfo(p["r_shoulder_swing_joint"])
rospy.sleep(2)

for i in range(3):
    negative = -1
    value = 0.7508
    if i % 2 == 0:
        negative = 1
    else:
        negative = -1

    angle = {
        "l_shoulder_swing_joint" : value*negative,
        "r_shoulder_swing_joint" : value*negative
    }

    typea.set_angles_slow(angle)

angle = {
    "l_shoulder_swing_joint" : 1.5708,
    "r_shoulder_swing_joint" : -1.5708
    }

typea.set_angles_slow(angle)

