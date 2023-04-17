# -*- coding: utf-8 -*-
#이승헌
#kobuki_mic_noi_led + obstacle_avoidance (1).py 내용 합쳤습니다.
#작성자 : 이승헌_아이포_
# pocketsphinx를 사용해서 이동을 제어하는데, 장애물 회피 내용이 없어 obstacle_avoidance내용을 합쳤습니다. 
# x,y 좌표 액션토픽으로 추가했습니다.
# 버튼 2번 종료 내용 추가. _이승헌
#!/usr/bin/env python
# 샘플레이트 테스트 완료 _이승헌
# 4.10 대화형으로 만들어보기 _이승헌
# 4.12 대화형으로 만들기 위해서 수정 _ 테스트 _이승헌
# 4.13 mp3 파일을 종료시키기 위해서 subprocess추가함

import os
import rospy
import numpy as np
import pyaudio
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from pocketsphinx import LiveSpeech, get_model_path
from kobuki_msgs.msg import Sound
import math
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from gtts import gTTS
from std_msgs.msg import Empty
import threading
import subprocess
def play_and_wait(file):
    subprocess.call(['mpg321', file])
rospy.init_node('pocket_sphinx_controller', anonymous=True)
min_distance = float('inf')
kobuki_velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
sphinx_output = None
user_response_event = threading.Event()
action_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)  # 수정된 부분


def play_and_wait(file):
    subprocess.call(['mpg321', file])

def laser_scan_callback(scan):
    global min_distance
    min_distance = min(scan.ranges)
# 사용자의 응답을 받는 콜백 함수
def user_response_callback(msg):
    global user_response
    global user_response_event 
    user_response = msg.data.lower().strip()
    user_response_event.set() #사용자 응답이 도착했음을 알리기 위해 이벤트 설정하기.


def process_sphinx_result(sphinx_output, robot_recognized):
    global user_response
    global user_response_event
    user_response_event.clear()
    kobuki_twist = Twist()
    while True:
        sphinx_output = get_sphinx_output()  # 사용자의 응답을 기다림
        if robot_recognized:
            if sphinx_output == "home" :
                # gTTS로 음성 출력
                tts = gTTS(text='home 구역으로 가는 것이 맞습니까?', lang='ko')
                tts.save('home_question.mp3')

                # 음성 파일을 별도의 스레드에서 재생
                play_thread = threading.Thread(target=play_and_wait, args=('home_question.mp3',))
                play_thread.start()

                user_response = get_sphinx_output()  # 사용자의 응답을 기다림

                # 사용자 응답이 있으면, 재생 스레드가 끝날 때까지 기다림
                play_thread.join()

                if user_response.lower() == "yes":
                    # home이 출력되면 x=0.0, y=0.0 좌표로 이동하는 액션토픽 전송
                    action_msg = MoveBaseActionGoal()
                    action_msg.goal.target_pose.header.frame_id = "map"
                    action_msg.goal.target_pose.header.stamp = rospy.Time.now()
                    action_msg.goal.target_pose.pose.position.x = 6.0
                    action_msg.goal.target_pose.pose.position.y = 3.5
                    action_msg.goal.target_pose.pose.orientation.w = 1.0
                    action_publisher.publish(action_msg)
                    robot_recognized = False
                elif user_response.lower() == "no":
                    print("사용자의 응답에 따라 이동하지 않습니다.")
                else:
                    print("알 수 없는 명령입니다. 다시 시도해주세요.")
            else:
                print("알 수 없는 명령입니다. 다시 시도해주세요.")           
                        
        elif sphinx_output == "robot":
            tts = gTTS(text='안녕하세요. 아이포 로봇입니다. 무엇을 도와드릴까요? 원하는 경로까지 안내를 해드리겠습니다.', lang='ko')
            tts.save('hello.mp3')
            os.system('mpg321 hello.mp3')
            robot_recognized = True
        else:
            return None, False
        rospy.sleep(3) # 3초간 정지 후 회전 동작 수행
        turn_msg = Twist()
        turn_msg.angular.z = 0.5 # 각속도를 0.5로 설정하여 좌회전 동작 수행
        rospy.sleep(3) # 3초간 좌회전을 수행한 후에 다시 이동 동작 수행
        kobuki_twist.linear.x = 0.5
        kobuki_twist.angular.z = 0.0    

        if min_distance < 0.5:  # 장애물이 0.5m 이내에 있는 경우
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
def callback(data):
    global robot_recognized
    sphinx_output = data.data
    kobuki_twist = process_sphinx_result(sphinx_output, robot_recognized) # robot_recognized 인자 추가
    if kobuki_twist is not None:
        kobuki_velocity_pub.publish(kobuki_twist)

dict_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "/home/chart/catkin_ws/src/my_dict.dict")
print("사용 중인 사전 파일 경로:", dict_path)

def user_response_listener():
    rospy.Subscriber('/user_response', String, user_response_callback)

if __name__ == '__main__':
    robot_recognized = False
    action_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
    user_response_listener()  # 수정된 부분
    
    rospy.Subscriber('/user_response', String, user_response_callback)
    rate = rospy.Rate(10)  # 10Hz
    rospy.Subscriber('/recognizer/output', String, callback)
 
    kobuki_twist = None
    user_response_thread = threading.Thread(target=user_response_listener)
    user_response_thread.start()
    while not rospy.is_shutdown():
        sphinx_output = get_sphinx_output() # 이 줄 잠깐 주석처리 0412
        if sphinx_output is None: 
            continue
        
        result = process_sphinx_result(sphinx_output, robot_recognized)
        if result is not None:
            kobuki_twist, robot_recognized = result
        if kobuki_twist is not None:
            kobuki_velocity_pub.publish(kobuki_twist)
            print("인식된 명령어:", sphinx_output)
        rate.sleep()
    rospy.spin()            
  
