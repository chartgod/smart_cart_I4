# -*- coding: utf-8 -*-
import os
import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from pocketsphinx import LiveSpeech, get_model_path
from scipy.fft import fft, ifft

rospy.init_node('pocket_sphinx_controller', anonymous=True)
kobuki_velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

# a, b, c (x, y)pose check
def process_sphinx_result(sphinx_output):
    kobuki_twist = Twist()

    if sphinx_output == "a":
        kobuki_twist.linear.x = 0.3
        kobuki_twist.angular.z = 0.0
    elif sphinx_output == "b":
        kobuki_twist.linear.x = -0.5
        kobuki_twist.angular.z = 0.0
    elif sphinx_output == "c":
        kobuki_twist.linear.x = 0.0
        kobuki_twist.angular.z = 1.0
    else:
        kobuki_twist.linear.x = 0.0
        kobuki_twist.angular.z = 0.0

    return kobuki_twist

def get_sphinx_output():
    model_path = get_model_path()
    speech = LiveSpeech(
        verbose=False,
        sampling_rate=44100, #16000, 2048 use O
        buffer_size=4096,
        no_search=False,
        full_utt=False,
        hmm=os.path.join(model_path, '/home/chart/.local/lib/python3.6/site-packages/pocketsphinx/model/en-us/en-us'),
        lm=os.path.join(model_path, '/home/chart/.local/lib/python3.6/site-packages/pocketsphinx/model/en-us/en-us.lm.bin'),
        dic=os.path.join(model_path,'/home/chart/catkin_ws/src/mic/my_dict.dict')
    )
    
    for phrase in speech:
        return str(phrase).lower().strip()

def remove_noise(audio_data):
    # FFT
    audio_data_freq = fft(audio_data)
    freq = np.fft.fftfreq(audio_data_freq.shape[-1])
   
    # Noise Reduction
    noise_threshold = 0.5 
    audio_data_freq[np.abs(freq) > noise_threshold] = 0
    
    # Inverse FFT
    filtered_audio_data = ifft(audio_data_freq)
    return np.real(filtered_audio_data).astype(np.int16)
    
def callback(data):
    sphinx_output = data.data
    kobuki_twist = process_sphinx_result(sphinx_output)
    kobuki_velocity_pub.publish(kobuki_twist)

dict_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "/home/chart/catkin_ws/src/mic/my_dict.dict")
print("사용 중인 사전 파일 경로:", dict_path)

if __name__ == '__main__':
    rate = rospy.Rate(10)  # 10Hz
    rospy.Subscriber('/recognizer/output', String, callback)
    while not rospy.is_shutdown():
        sphinx_output = get_sphinx_output()
        audio_data = remove_noise(audio_data)  # 수정: 노이즈 제거 추가
        kobuki_twist = process_sphinx_result(sphinx_output)
        kobuki_velocity_pub.publish(kobuki_twist)
        rate.sleep()
    rospy.spin()
#remove_noise()함수를 이용해서 FFT를 이용한 노이즈 제거.

# audio data를 입력 받고나서  FFT를 이용한 노이즈 제거를 수행하고, 노이즈가 제거된 audio data를 반환합니다. 

#추가적으로, 노이즈 제거에 대한 실험적인 설정인 noise_threshold 값은 일반적으로는 0.01~0.1 사이의 값으로 설정합니다.

# 이 값은 노이즈가 포함되어 있는 주파수 대역을 결정하는데 사용되며, 너무 높은 값으로 설정하면 음성 신호까지 제거되는 경우가 있으니 잘 선택.
