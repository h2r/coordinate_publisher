#!/usr/bin/env python

import time
import sys
import roslib
roslib.load_manifest("coordinate_publisher")
import rospy
import tf
from coordinate_publisher.msg import PointArray
from std_msgs.msg import Header, String, Float32MultiArray, Bool
from tf.transformations import quaternion_inverse, quaternion_matrix
from geometry_msgs.msg import Point
from object_recognition_msgs.msg import RecognizedObjectArray

global left_arm_origin
global right_arm_origin
global left_arm_point
global right_arm_point
global frame
frame = "/reference/base"

def sub_vec(v1,v2):
    ret = []
    for i in range(len(v1)):
        ret += [v1[i]-v2[i]]
    return tuple(ret)
def add_vec(v1,v2):
    ret = []
    for i in range(len(v1)):
        ret += [v1[i]+v2[i]]
    return tuple(ret)
#fills body points from openni data
def fill_points(tfl):
    try:
        global user
        allFramesString = tfl.getFrameStrings()
        onlyUsers = set([line for line in allFramesString if 'right_elbow_' in line])
        n = len('right_elbow_')
        userIDs = [el[n:] for el in onlyUsers]
        user = ''
        if len(userIDs) > 0:
            mostRecentUID = userIDs[0]
            mostRecentTime = tfl.getLatestCommonTime(frame, 'right_elbow_' + mostRecentUID).to_sec()
            for uid in userIDs:
                compTime = tfl.getLatestCommonTime(frame, 'right_elbow_' + uid).to_sec()
                #rospy.loginfo("Diff time " + str(rospy.get_rostime().to_sec() - compTime))
                if compTime >= mostRecentTime and rospy.get_rostime().to_sec() - compTime < 5:
                    user = uid
                    mostRecentTime = compTime
        global left_arm_origin
        global right_arm_origin
        global head_origin
        global head_point
        global left_arm_point
        global right_arm_point
        global left_foot
        global right_foot
        (to_left_elbow,_) = tfl.lookupTransform(frame,"/left_elbow_" + user, rospy.Time(0))
        (to_right_elbow,_) = tfl.lookupTransform(frame,"/right_elbow_" + user, rospy.Time(0))
        (to_left_hand,_) = tfl.lookupTransform(frame,"/left_hand_" + user, rospy.Time(0))
        (to_right_hand,_) = tfl.lookupTransform(frame,"/right_hand_" + user, rospy.Time(0))
        (right_foot,_) = tfl.lookupTransform(frame, "/right_foot_" + user, rospy.Time(0))
        (left_foot,_) = tfl.lookupTransform(frame, "/left_foot_" + user, rospy.Time(0))
        (to_head,head_rot) = tfl.lookupTransform(frame,"/head_" + user, rospy.Time(0))
        left_arm_origin = to_left_hand
        left_arm_point = add_vec(to_left_hand, sub_vec(to_left_hand, to_left_elbow))
        right_arm_origin = to_right_hand
        right_arm_point = add_vec(to_right_hand, sub_vec(to_right_hand, to_right_elbow))
        return True
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        left_arm_point = None
        right_arm_origin = None
        left_arm_origin = None
        right_arm_point = None
        head_point = None
        head_origin = None
        left_foot = None
        right_foot = None
        return False

def main():
    global tfl
    rospy.init_node('coordinate_publisher')
    tfl = tf.TransformListener()
    array_publisher = rospy.Publisher('all_positions', PointArray, queue_size=1)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():       
        if(fill_points(tfl)):
            lo = Point()
            ro = Point()
            lp = Point()
            rp = Point()
            rp.x = right_arm_point[0]
            rp.y = right_arm_point[1]
            rp.z = right_arm_point[2]
            lp.x = left_arm_point[0]
            lp.y = left_arm_point[1]
            lp.z = left_arm_point[2]
            ro.x = right_arm_origin[0]
            ro.y = right_arm_origin[1]
            ro.z = right_arm_origin[2]
            lo.x = left_arm_origin[0]
            lo.y = left_arm_origin[1]
            lo.z = left_arm_origin[2]
            array_publisher.publish((lo, ro, lp, rp))
        rate.sleep()


if __name__ == '__main__':
    main()
