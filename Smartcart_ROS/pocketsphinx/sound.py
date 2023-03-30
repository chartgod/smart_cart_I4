# -*- coding: utf-8 -*-
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