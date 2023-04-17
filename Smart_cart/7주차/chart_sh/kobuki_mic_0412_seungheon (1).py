# -*- coding: utf-8 -*-
#kobuki_mic_noi_led + obstacle_avoidance (1).py 내용 합쳤습니다.
#작성자 : 이승헌_아이포_
# pocketsphinx를 사용해서 이동을 제어하는데, 장애물 회피 내용이 없어 obstacle_avoidance내용을 합쳤습니다. 
# x,y 좌표 액션토픽으로 추가했습니다.
# 버튼 2번 종료 내용 추가. _이승헌
#!/usr/bin/env python
# 샘플레이트 테스트 완료 _이승헌
# 4.10 대화형으로 만들어보기 _이승헌
# -*- coding: utf-8 -*-
import os
import rospy
import numpy as np
import pyaudio
import matplotlib.pyplot as plt
import serial
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from pocketsphinx import LiveSpeech, get_model_path
from kobuki_msgs.msg import Sound
import math
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from gtts import gTTS
from std_msgs.msg import Empty
rospy.init_node('pocket_sphinx_controller', anonymous=True)
min_distance = float('inf')
kobuki_velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
robot_recognized = False

def callback(data):
    sphinx_output = data.data
    kobuki_twist = process_sphinx_result(sphinx_output)
    kobuki_velocity_pub.publish(kobuki_twist)

def laser_scan_callback(scan):
    global min_distance
    min_distance = min(scan.ranges)
# 사용자의 응답을 받는 콜백 함수
def user_response_callback(msg):
    global user_response
    user_response = msg.data.lower().strip()
    user_response = None
    rospy.sleep(1) # 사용자의 응답을 저장하는 전역 변수
   
#a,c (x,y)pose check
# a 또는 a point 출력 시 gTTS 파일로 물어보는 코드 추가
def process_sphinx_result(sphinx_output, robot_recognized):
    global user_response
    kobuki_twist = Twist()

    if robot_recognized:
        if sphinx_output == "robot":
            tts = gTTS(text='안녕하세요. 아이포 로봇입니다. 무엇을 도와드릴까요? 원하는 경로까지 안내를 해드리겠습니다.', lang='ko')
            tts.save('hello.mp3')
            os.system('mpg321 hello.mp3')
            # 사용자의 응답을 받기 위한 코드
            while user_response is None:
                rospy.sleep(0.5)
            user_response = None 
            return None, False  #I4를 인식했음을 알립니다.
        elif sphinx_output in ['a', 'a point']:
            # a 구역으로 가는지 묻는 음성 출력
            tts = gTTS(text='a 구역으로 가는 것이 맞습니까?', lang='ko')
            tts.save('A_question.mp3')
            os.system('mpg321 A_question.mp3')

            # 사용자의 응답을 받기 위한 코드
            while user_response is None:
                rospy.sleep(0.5)

            if user_response.lower() in ['yes', 'a c']:
                # a 구역으로 이동하는 코드
                action_msg = MoveBaseActionGoal()
                action_msg.goal.target_pose.header.frame_id = "map"
                action_msg.goal.target_pose.header.stamp = rospy.Time.now()
                action_msg.goal.target_pose.pose.position.x = 6.5
                action_msg.goal.target_pose.pose.position.y = 0.5
                action_msg.goal.target_pose.pose.orientation.w = 1.0

                # a 구역에 도착했다는 음성 출력
                tts = gTTS(text='아이포 로봇이 a 구역에 도착했습니다.', lang='ko')
                tts.save('A1.mp3')
                os.system('mpg321 A1.mp3')

                action_publisher.publish(action_msg)
            else:
                print("사용자의 응답에 따라 이동하지 않습니다.")
    # b가 출력되면 x=6.5, y=2.5 좌표로 이동하는 액션토픽 전송
        elif sphinx_output in ['b', 'b point']:
            # a 구역으로 가는지 묻는 음성 출력
            tts = gTTS(text='b 구역으로 가는 것이 맞습니까?', lang='ko')
            tts.save('B_question.mp3')
            os.system('mpg321 B_question.mp3')

            # 사용자의 응답을 받기 위한 코드
            while user_response is None:
                rospy.sleep(0.5)

            if user_response.lower() in ['yes', 'a c']:
                # a 구역으로 이동하는 코드
                action_msg = MoveBaseActionGoal()
                action_msg.goal.target_pose.header.frame_id = "map"
                action_msg.goal.target_pose.header.stamp = rospy.Time.now()
                action_msg.goal.target_pose.pose.position.x = 6.5
                action_msg.goal.target_pose.pose.position.y = 2.5
                action_msg.goal.target_pose.pose.orientation.w = 1.0

                # a 구역에 도착했다는 음성 출력
                tts = gTTS(text='아이포 로봇이 b 구역에 도착했습니다.', lang='ko')
                tts.save('B1.mp3')
                os.system('mpg321 B1.mp3')

                action_publisher.publish(action_msg)
            else:
                print("사용자의 응답에 따라 이동하지 않습니다.")
    # c가 출력되면 x=2.0, y=5.8 좌표로 이동하는 액션토픽 전송
        elif sphinx_output in ['c', 'c point']:
            # a 구역으로 가는지 묻는 음성 출력
            tts = gTTS(text='c 구역으로 가는 것이 맞습니까?', lang='ko')
            tts.save('C_question.mp3')
            os.system('mpg321 C_question.mp3')

            # 사용자의 응답을 받기 위한 코드
            while user_response is None:
                rospy.sleep(0.5)

            if user_response.lower() in ['yes', 'a c']:
                # a 구역으로 이동하는 코드
                action_msg = MoveBaseActionGoal()
                action_msg.goal.target_pose.header.frame_id = "map"
                action_msg.goal.target_pose.header.stamp = rospy.Time.now()
                action_msg.goal.target_pose.pose.position.x = 2.0
                action_msg.goal.target_pose.pose.position.y = 5.8
                action_msg.goal.target_pose.pose.orientation.w = 1.0

                # a 구역에 도착했다는 음성 출력
                tts = gTTS(text='아이포 로봇이 c 구역에 도착했습니다.', lang='ko')
                tts.save('C1.mp3')
                os.system('mpg321 C1.mp3')

                action_publisher.publish(action_msg)
            else:
                print("사용자의 응답에 따라 이동하지 않습니다.")
        elif sphinx_output in ['home']:
            # a 구역으로 가는지 묻는 음성 출력
            tts = gTTS(text='home 구역으로 가는 것이 맞습니까?', lang='ko')
            tts.save('Home_question.mp3')
            os.system('mpg321 Home_question.mp3')

            # 사용자의 응답을 받기 위한 코드
            while user_response is None:
                rospy.sleep(0.5)

            if user_response.lower() in ['yes', 'a c']:
                # a 구역으로 이동하는 코드
                action_msg = MoveBaseActionGoal()
                action_msg.goal.target_pose.header.frame_id = "map"
                action_msg.goal.target_pose.header.stamp = rospy.Time.now()
                action_msg.goal.target_pose.pose.position.x = 0.0
                action_msg.goal.target_pose.pose.position.y = 0.5
                action_msg.goal.target_pose.pose.orientation.w = 1.0

                # a 구역에 도착했다는 음성 출력
                tts = gTTS(text='아이포 로봇이 home 구역에 도착했습니다.', lang='ko')
                tts.save('Home.mp3') #파일 이름 제대로 확인
                os.system('mpg321 Home.mp3')

                action_publisher.publish(action_msg)
            else:
                print("사용자의 응답에 따라 이동하지 않습니다.")
            user_response = None
    if min_distance < 0.5:
        kobuki_twist.linear.x = 0.0
        kobuki_twist.angular.z = 0.5  # 좌회전
    else:
        kobuki_twist.linear.x = 0.0
        kobuki_twist.angular.z = 0.0

    return kobuki_twist, robot_recognized

    


def get_sphinx_output():
    model_path = get_model_path()
    speech = LiveSpeech(
        verbose=False,
        sampling_rate=20000, #16000, 2048 use O
        buffer_size=2048,
        no_search=False,
        full_utt=False,
        hmm=os.path.join(model_path, '/home/chart/.local/lib/python3.8/site-packages/pocketsphinx/model/en-us/en-us'),
        lm=os.path.join(model_path, '/home/chart/.local/lib/python3.8/site-packages/pocketsphinx/model/en-us/en-us.lm.bin'),
        dic=os.path.join(model_path,'/home/chart/catkin_ws/src/my_dict.dict')
    )
    print("음성 인식을 시작합니다. a, b, c, home, a point, b point, c point, robot 중 하나를 말해주세요.")
    for phrase in speech:
        recognized_phrase = str(phrase).lower().strip()
        if recognized_phrase in ['a', 'b', 'c', 'home', 'a point', 'b point', 'c point', 'robot']:
            print("인식된 명령어:", recognized_phrase)
            return recognized_phrase
        else:
            print("인식되지 않은 명령어:", recognized_phrase)    

    

dict_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "/home/chart/catkin_ws/src/my_dict.dict")
print("사용 중인 사전 파일 경로:", dict_path)

if __name__ == '__main__':
    action_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
    rospy.Subscriber('/user_response', String, user_response_callback)
    rate = rospy.Rate(10)  # 10Hz
    rospy.Subscriber('/recognizer/output', String, callback)
    robot_recognized = False  # I4 인식 여부를 추적하는 변수
    kobuki_twist = None
    while not rospy.is_shutdown():
        sphinx_output = get_sphinx_output()
        if sphinx_output is None: continue
        result = process_sphinx_result(sphinx_output, robot_recognized)
        if result is not None:
            kobuki_twist, robot_recognized = result
        if kobuki_twist is not None:
            kobuki_velocity_pub.publish(kobuki_twist)
            print("인식된 명령어:", sphinx_output)
        rate.sleep()
    rospy.spin()

