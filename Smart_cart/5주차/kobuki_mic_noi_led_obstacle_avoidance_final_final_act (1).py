#kobuki_mic_noi_led + obstacle_avoidance (1).py 내용 합쳤습니다.
#작성자 : 이승헌_아이포_
# pocketsphinx를 사용해서 이동을 제어하는데, 장애물 회피 내용이 없어 obstacle_avoidance내용을 합쳤습니다. 
# x,y 좌표 액션토픽으로 추가했습니다.
# 버튼 2번 종료 내용 추가. _이승헌
# gtts 내용 추가하기 

#!/usr/bin/env python
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
from scipy.fft import fft, ifft
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import Sound
import math
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from gtts import gTTS

LED_PIN = 8
button_pressed = False
ser = serial.Serial('/dev/ttyACM0', 9600)

def process_sphinx_result(sphinx_output):
    kobuki_twist = Twist()

    if sphinx_output == "a":
    # a가 출력되면 x=6.5, y=0.5 좌표로 이동하는 액션토픽 전송
        action_msg = MoveBaseActionGoal()
        action_msg.goal.target_pose.header.frame_id = "map"
        action_msg.goal.target_pose.header.stamp = rospy.Time.now()
        action_msg.goal.target_pose.pose.position.x = 6.5
        action_msg.goal.target_pose.pose.position.y = 0.5
        action_msg.goal.target_pose.pose.orientation.w = 1.0
        # gtts로 음성 출력
        tts = gTTS(text='아이포 로봇이 a구역에 도착했습니다.', lang='ko') #추가
        tts.save('A1.mp3')
        os.system('mpg321 A1.mp3')
        
    elif sphinx_output == "b":
    # a가 출력되면 x=6.5, y=0.5 좌표로 이동하는 액션토픽 전송
        action_msg = MoveBaseActionGoal()
        action_msg.goal.target_pose.header.frame_id = "map"
        action_msg.goal.target_pose.header.stamp = rospy.Time.now()
        action_msg.goal.target_pose.pose.position.x = 6.5
        action_msg.goal.target_pose.pose.position.y = 5.5
        action_msg.goal.target_pose.pose.orientation.w = 1.0
        
    elif sphinx_output == "c":
    # c가 출력되면 x=6.5, y=0.5 좌표로 이동하는 액션토픽 전송
        action_msg = MoveBaseActionGoal()
        action_msg.goal.target_pose.header.frame_id = "map"
        action_msg.goal.target_pose.header.stamp = rospy.Time.now()
        action_msg.goal.target_pose.pose.position.x = 2.0
        action_msg.goal.target_pose.pose.position.y = 5.8
        action_msg.goal.target_pose.pose.orientation.w = 1.0
        
    elif sphinx_output== "d":
    # a가 출력되면 x=6.5, y=0.5 좌표로 이동하는 액션토픽 전송
        action_msg = MoveBaseActionGoal()
        action_msg.goal.target_pose.header.frame_id = "map"
        action_msg.goal.target_pose.header.stamp = rospy.Time.now()
        action_msg.goal.target_pose.pose.position.x = 0.0
        action_msg.goal.target_pose.pose.position.y = 0.0
        action_msg.goal.target_pose.pose.orientation.w = 1.0
    
        rospy.sleep(3) # 3초간 정지 후 회전 동작 수행
        turn_msg = Twist()
        turn_msg.angular.z = 0.5 # 각속도를 0.5로 설정하여 좌회전 동작 수행
        cmd_vel_pub.publish(turn_msg)
        rospy.sleep(3) # 3초간 좌회전을 수행한 후에 다시 이동 동작 수행
        kobuki_twist.linear.x = 0.5
        kobuki_twist.angular.z = 0.0
    
    else:
        kobuki_twist.linear.x = 0.0
        kobuki_twist.angular.z = 0.0

    return kobuki_twist

def get_sphinx_output():
    model_path = get_model_path()
    speech = LiveSpeech(
        verbose=False,
        sampling_rate=44100,
        buffer_size=4096,
        no_search=False,
        full_utt=False,
        hmm=os.path.join(model_path, '/home/chart/.local/lib/python3.8/site-packages/pocketsphinx/model/en-us/en-us'),
        lm=os.path.join(model_path, '/home/chart/.local/lib/python3.8/site-packages/pocketsphinx/model/en-us/en-us.lm.bin'),
        dic=os.path.join(model_path,'/home/chart/catkin_ws/src/mic/my_dict.dict')
    )
    results = []
    for phrase in speech:
        result = str(phrase).lower() #result = str(phrase).lower().strip() 공백 제거 내용이 없음.
        results.append(result)
        print("음성 인식 결과: {}".format(result))
    return results

def remove_noise(audio_data):
    audio_data_freq = fft(audio_data)
    freq = np.fft.fftfreq(audio_data_freq.shape[-1])
   
    noise_threshold = 0.5 
    audio_data_freq[np.abs(freq) > noise_threshold] = 0
    
    filtered_audio_data = ifft(audio_data_freq)
    
    fig, ax = plt.subplots()
    ax.plot(freq, np.abs(audio_data_freq))
    ax.set_xlim([-0.1, 0.1])
    ax.set_ylim([0, 300000])
    ax.set_xlabel('Frequency [Hz]')
    ax.set_ylabel('Amplitude')
    plt.show()
    
    return np.real(filtered_audio_data).astype(np.int16)

def callback(data):
    distances = data.ranges
    angles = [math.radians(i) for i in range(len(distances))]
    
    closest_distance = [d for d in distances if d > 0.0]
    min_distance = min(closest_distance)
    closest_angle = angles[distances.index(min(closest_distance))]
    
    x = min_distance * math.cos(closest_angle)
    y = min_distance * math.sin(closest_angle)
    print("Object coordinates: ({}, {})".format(x, y))
    print('distances', min_distance)
    
    sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=10)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    if min_distance < 0.3:
        sound_msg = Sound()
        sound_msg.value = Sound.ON
        sound_pub.publish(sound_msg)
        rospy.loginfo("Child detected! Warning!, I'm I4 robot. We put safety first.")
        
        stop_msg = Twist()
        cmd_vel_pub.publish(stop_msg)
        
        turn_msg = Twist()
        turn_msg.angular.z = 0.5
        cmd_vel_pub.publish(turn_msg)

rospy.init_node('pocket_sphinx_controller', anonymous=True)
laser_sub = rospy.Subscriber('/scan', LaserScan, callback)
kobuki_velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100

p = pyaudio.PyAudio()

stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)

# 버튼 눌린 횟수를 저장할 변수 초기화
button_count = 0

while not button_pressed:
    button_state = ser.readline().decode().strip()
    if button_state == '1':
        button_pressed = True
        ser.write(bytes(str(LED_PIN) + 'H', 'utf-8'))
        break
    rospy.sleep(0.1)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    audio_data = stream.read(CHUNK)
    audio_data = np.frombuffer(audio_data, dtype=np.int16)
    audio_data = remove_noise(audio_data)
    sphinx_output = get_sphinx_output()
    kobuki_twist = process_sphinx_result(sphinx_output)
    kobuki_velocity_pub.publish(kobuki_twist)
    
    if button_pressed:
        ser.write(bytes(str(LED_PIN) + 'H', 'utf-8'))
        
        # 버튼을 2번 누르면 이동을 종료하도록 처리
        button_state = ser.readline().decode().strip()
        if button_state == '1':
            button_count += 1
        else:
            button_count = 0
        
        if button_count >= 2:
            break
    
    rate.sleep()

if button_pressed:
    ser.write(bytes(str(LED_PIN) + 'L', 'utf-8'))

stream.stop_stream()
stream.close()
p.terminate()
rospy.spin()
