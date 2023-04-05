#!/bin/bash
import rospy
import move_base_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

import numpy #base_position 문자열 정열에 사용
import sys #bash argument 받기 위해 추가

print(sys.argv[1])
print(sys.argv[2])

def test_client():

    rate = rospy.Rate(5)

    #ac = actionlib.simple_action_client("move_base",MoveBaseAction)
    _ac = actionlib.SimpleActionClient("move_base",MoveBaseAction)
    _ac.wait_for_server()

    goal = MoveBaseGoal()
    
    goal.target_pose.header.seq = 0
    goal.target_pose.header.stamp.secs = 0
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = float(sys.argv[1])
    goal.target_pose.pose.position.y = float(sys.argv[2])
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.025547
    goal.target_pose.pose.orientation.w = 0.98381429

    print("==== Sending Goal to Server ====")
    _ac.send_goal(goal, feedback_cb=test_feedback)

    _ac.wait_for_result()

    return _ac.get_result()

def list_float(list): # list -> float 변경 함수
    float_ = float(list[0])
    for i in range(1,16):
        float_ += ((1/(10**i)) * float(list[1+i]))
    return float_

def test_feedback(feedback):
    with open('base_position.txt','w',encoding='UTF-8') as f:
        f.write(str(feedback))
    fo = open('./base_position.txt','r')
    data = fo.read()
    fo.close()
    print(data)
    print("test")
    data = ' '.join(data).split() # 공백 제거
    #data = list(filter(None, data))
    data = numpy.array(data)
    index = numpy.where(data == ':') # ':' 요소 모두 찾기 index 반환 tuple
    index = list(index[0]) # 리스트로 변환
    
    position_x = data[index[9]+1:index[10]-1]
    position_y = data[index[10]+1:index[11]-1]
    orientation_z = data[index[15]+1:index[16]-1]
    orientation_w = data[index[16]+1:]
    ########### 각각 float 으로 변경 해야함 ###############

    position_x = list_float(position_x)
    position_y = list_float(position_y)
    orientation_z = list_float(orientation_z)
    orientation_w = list_float(orientation_w)
    
    print("position_x",position_x,type(position_x))
    print("position_y",position_y,type(position_y))
    print("orientation_z",orientation_z,type(orientation_z))
    print("orientation_w",orientation_w,type(orientation_w))

    #print(data)

    #move_base_msgs.msg._MoveBaseFeedback.MoveBaseFeedback.base_position.pose
    #print(feedback)
    #print(type(feedback))
    #dict1 = list(feedback)
    #print(type(dict1))
    
    
if __name__ == '__main__':
    try:
        rospy.init_node('action_client_py')
        result = test_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
