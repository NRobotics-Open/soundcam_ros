#! /usr/bin/env python3

import rospy, time, sys
from diagnostic_msgs.msg import KeyValue
from soundcam_ros.msg import SoundcamAction, SoundcamFeedback, SoundcamGoal, SoundcamResult
from actionlib import SimpleActionClient
import uuid

if __name__ == '__main__':
    try:
        rospy.init_node('sc_tst_srvr_node')
        client = SimpleActionClient('SoundCameraActionServer', SoundcamAction)
        client.wait_for_server()
        rospy.loginfo('Found action server!')

        goal = SoundcamGoal()
        goal.parameters.append(KeyValue(key='uuid', value=str(uuid.uuid4())))
        goal.parameters.append(KeyValue(key='delay', value=str(1)))
        goal.parameters.append(KeyValue(key='numCaptures', value=str(1)))
        goal.parameters.append(KeyValue(key='recordTime', value=str(15))) #seconds
        goal.parameters.append(KeyValue(key='mediaType', value=''))
        goal.parameters.append(KeyValue(key='missionId', value=str(16)))
        goal.parameters.append(KeyValue(key='missionName', value='test-snapshot'))
        goal.parameters.append(KeyValue(key='waypointId', value=str(34)))
        goal.parameters.append(KeyValue(key='waypointX', value=str(1.19)))
        goal.parameters.append(KeyValue(key='waypointY', value=str(0.98)))
        goal.parameters.append(KeyValue(key='waypointTheta', value=str(90.0)))
        goal.parameters.append(KeyValue(key='imageTileNo', value=str(0)))
        goal.parameters.append(KeyValue(key='currentLoop', value=str(0)))

        rospy.loginfo('Sending Snapshot goal!')
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        rospy.logwarn("Result:", )

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)



