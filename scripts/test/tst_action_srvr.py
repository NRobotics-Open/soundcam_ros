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
        uuid_str = uuid.uuid4()

        send_multiple = True

        def sendGoal(tile_no=0, recordTime=0, msnId=34):
            goal = SoundcamGoal()
            goal.parameters.append(KeyValue(key='uuid', value=str(uuid_str)))
            goal.parameters.append(KeyValue(key='delay', value=str(1)))
            goal.parameters.append(KeyValue(key='numCaptures', value=str(1)))
            goal.parameters.append(KeyValue(key='recordTime', value=str(recordTime))) #seconds
            goal.parameters.append(KeyValue(key='mediaType', value=str(5)))
            goal.parameters.append(KeyValue(key='missionId', value=str(msnId)))
            goal.parameters.append(KeyValue(key='missionName', value='test-snapshot'))
            goal.parameters.append(KeyValue(key='waypointId', value=str(34)))
            goal.parameters.append(KeyValue(key='waypointX', value=str(1.19)))
            goal.parameters.append(KeyValue(key='waypointY', value=str(0.98)))
            goal.parameters.append(KeyValue(key='waypointTheta', value=str(90.0)))
            goal.parameters.append(KeyValue(key='imageTileNo', value=str(tile_no)))
            goal.parameters.append(KeyValue(key='currentLoop', value=str(0)))
            goal.parameters.append(KeyValue(key='resultDirectory', value='/home/ephson/missionresults/16/2025_01_30/00_00'))

            rospy.loginfo('Sending Snapshot goal!')
            client.send_goal(goal)
            client.wait_for_result()
            result = client.get_result()
            rospy.logwarn("Result:", )

        if(send_multiple):
            for i in range(5):
                sendGoal(tile_no=i)
                time.sleep(1.5)
        else:
            sendGoal()
        
        print('Test completed!')
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)



