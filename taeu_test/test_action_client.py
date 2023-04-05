#!/bin/bash
import rospy
import move_base_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

import sys #bash argument 받기 위해 추가

print(sys.argv[1])
print(sys.argv[2])


PENDING = 0    # 목표가 아직 액션 서버에 의해 처리되지 않음
ACTIVE = 1    # 목표가 현재 액션 서버에 의해 처리되고 있음
PREEMPTED = 2    # 목표가 실행을 시작한 후 취소 요청을 받았습니다. 이후 실행을 완료했습니다(터미널 상태)
SUCCEEDED = 3    # 작업 서버가 목표를 성공적으로 달성했습니다(터미널 상태)
ABORTED = 4    # 일부 오류로 인해 작업 서버가 실행 중에 목표를 중단했습니다 (터미널 상태)
REJECTED = 5   # 목표가 처리되지 않고 작업 서버에 의해 거부되었습니다. 목표가 달성 불가능하거나 유효하지 않기 때문입니다(터미널 상태)
PREEMPTING = 6    # 목표가 실행을 시작한 후 취소 요청을 받았고 아직 실행을 완료하지 않았습니다.
RECALLING = 7    # 목표가 실행을 시작하기 전에 취소 요청을 받았지만 작업 서버에서 아직 목표가 취소되었는지 확인하지 않았습니다.
RECALLED = 8    # 목표가 실행을 시작하기 전에 취소 요청을 받았고 #성공적으로 취소되었습니다(터미널 상태 )
LOST = 9    # 액션 클라이언트는 목표가 LOST인지 확인할 수 있습니다. 이러면 안 된다. 작업 서버에서 유선으로 전송

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
    _ac.send_goal(goal)#, feedback_cb=test_feedback)

    state_result = _ac.get_state()
    print(state_result)
    #move_base_msgs.msg._MoveBaseFeedback.MoveBaseFeedback.base_position.pose
    while state_result < PREEMPTED:
        rate.sleep()
        state_result = _ac.get_state()
        print(state_result)

    if state_result == SUCCEEDED:
        rospy.logwarn("Action Done State Result : ")
        print(_ac.get_result)
    else:
        rospy.logerr("Something went wrong, result state : " + str(state_result))
    _ac.wait_for_result()

    return _ac.get_result()

def test_feedback(feedback):
    print(feedback)

if __name__ == '__main__':
    try:
        rospy.init_node('action_client_py')
        result = test_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
