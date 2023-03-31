# -*- coding: utf-8 -*-
import os
import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from pocketsphinx import LiveSpeech, get_model_path
from scipy.fft import fft, ifft
import pyaudio
import matplotlib.pyplot as plt

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
    results = []
    for phrase in speech:
        result = str(phrase).lower().strip()
        results.append(result)
        print("음성 인식 결과: {}".format(result))
    return results

def remove_noise(audio_data):
    # FFT
    audio_data_freq = fft(audio_data)
    freq = np.fft.fftfreq(audio_data_freq.shape[-1])
   
    # Noise Reduction
    noise_threshold = 0.5 
    audio_data_freq[np.abs(freq) > noise_threshold] = 0
    
    # Inverse FFT
    filtered_audio_data = ifft(audio_data_freq)
    
    
    # Plot FFT result
    fig, ax = plt.subplots()
    ax.plot(freq, np.abs(audio_data_freq))
    ax.set_xlim([-0.1, 0.1])
    ax.set_ylim([0, 300000])
    ax.set_xlabel('Frequency [Hz]')
    ax.set_ylabel('Amplitude')
    plt.show()
    
    return np.real(filtered_audio_data).astype(np.int16)

CHUNK = 1024 #버퍼 크기, 음성 데이터를 몇 분할해서 사용할 지.
FORMAT = pyaudio.paInt16 #샘플링 데이터형식 16비트
CHANNELS = 1 #모노 수 1이면 모노, 2 스테레오
RATE = 44100 #레이트 샘플링 크기

p = pyaudio.PyAudio()

stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)

if __name__ == '__main__':
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        audio_data = stream.read(CHUNK)
        audio_data = np.frombuffer(audio_data, dtype=np.int16)
        audio_data = remove_noise(audio_data)
        sphinx_output = get_sphinx_output()
        kobuki_twist = process_sphinx_result(sphinx_output)
        kobuki_velocity_pub.publish(kobuki_twist)  # 추가: 발행 코드
        rate.sleep()
    stream.stop_stream()
    stream.close()
    p.terminate()
    rospy.spin()
