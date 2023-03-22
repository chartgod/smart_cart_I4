# -*- coding: utf-8 -*-
'''  if distances > 0.0:
        closest_distance = min(distances)
        print('distances',closest_distance)
        '''
import rospy
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import Sound
from geometry_msgs.msg import Twist
import math

def callback(data):
    # 라이다에서 측정한 거리 정보를 가져옴 + angles을 추가해서 각도를 가져 왔음.
    distances = data.ranges
    #리스트를 값을 줘도 안 나옴, closet_angle = angles에 []대신 ()로 바꿨는데도 오류가 발생.
    angles = [math.radians(i) for i in range(len(distances))]
#     print(len(angles))
    # print(type(distances)) # print(list(distances))
    
    # 거리 정보에서 가장 가까운 값 추출 / 거리 내용만 추출했는데 angle추가함.
    closest_distance = [ d for d in distances if d > 0.0 ]
    min_distance = min(closest_distance) #추가
#     closest_angle = angles[distances.index(min(closest_distance))] * 180/1000
    closest_angle = (1 + distances.index(min(closest_distance))) * 180/1000
    closest_angle_radians = math.radians(closest_angle)
    # 객체의 좌표 계산 및 distnaces 간격에 대한 내용 출력하기.
    x = min_distance * math.cos(closest_angle_radians) #수정
    y = min_distance * math.sin(closest_angle_radians)
    
    #closest_distance = min(closest_distance)
    sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=10)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # 가장 가까운 값이 일정 거리 이하일 때 경고 메시지 출력
    if min_distance < 0.5: #0.3m 안에 있을 경우에는 감지를 한다.
        print("angle", closest_angle)
        print("Object coordinates: ({}, {})".format(x, y))
        print('distances', min_distance)
        # sound_msg = Sound()
        # sound_msg.value = Sound.ON  # 사운드 메시지를 발행하기 위해 "ON" 값을 설정
        # sound_pub.publish(sound_msg)  # 발행
        
        rospy.loginfo("Child detected! Warning!, I'm I4 robot. We put safety first.")
        
        # 로봇 이동 멈춤
        stop_msg = Twist()
        cmd_vel_pub.publish(stop_msg)
        
        # 장애물을 피하기 위해 로봇 회전
        turn_msg = Twist()
        turn_msg.linear.x = -0.5
        # turn_msg.angular.z = -0.5  #아이포 로봇은 반시계 방향으로 회전한다.
        cmd_vel_pub.publish(turn_msg)
        
rospy.init_node('obstacle_avoidance_node')
laser_sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
