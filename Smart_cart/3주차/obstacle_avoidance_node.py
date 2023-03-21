"""
I4_이승헌

ref _ http://wiki.ros.org/sensor_msgs
http://wiki.ros.org/kobuki_msgs
https://github.com/yujinrobot/kobuki_msgs
import rospy
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import Sound

rospy.init_node('child_warning_node')

sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=10)
laser_sub = rospy.Subscriber('/scan', LaserScan, callback)

def callback(data):
    # 라이다에서 측정한 거리 정보를 가져옴
    distances = data.ranges
    
    # 거리 정보에서 가장 가까운 값 추출
    closest_distance = min(distances)
    
    # 가장 가까운 값이 일정 거리 이하일 때 경고 메시지 출력
    if closest_distance < 0.5:
        sound_pub.publish(0x6)
        rospy.loginfo("Child detected! Warning!")
sound_msg = Sound()
sound_msg.value = 1
rospy.spin()
위 내용은 경고메시지만 출력하는것."""
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import Sound, Twist

def callback(data):
    # 라이다에서 측정한 거리 정보를 가져옴
    distances = data.ranges
    
    # 거리 정보에서 가장 가까운 값 추출
    closest_distance = min(distances)
    
    sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=10)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # 가장 가까운 값이 일정 거리 이하일 때 경고 메시지 출력
    if closest_distance < 0.5: #0.5m 안에 있을 경우에는 감지를 한다.
        sound_msg = Sound()
        sound_msg.value = Sound.ON  # 사운드 메시지를 발행하기 위해 "ON" 값을 설정
        sound_pub.publish(sound_msg)  # 발행
        
        rospy.loginfo("Child detected! Warning!, I'm I4 robot. We put safety first.")
        
        # 로봇 이동 멈춤
        stop_msg = Twist()
        cmd_vel_pub.publish(stop_msg)
        
        # 장애물을 피하기 위해 로봇 회전
        turn_msg = Twist()
        turn_msg.angular.z = 1.0  #아이포 로봇은 반시계 방향으로 회전한다.
        cmd_vel_pub.publish(turn_msg)
        
rospy.init_node('obstacle_avoidance_node')
laser_sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
