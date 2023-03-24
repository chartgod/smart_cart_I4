# -*- coding: utf-8 -*-

import rospy
from kobuki_msgs.msg import SensorState
import cv2
import numpy as np

img_height, img_width = 100, 300
img = np.zeros((img_height, img_width, 3), np.uint8)

def battery_callback(msg):
    battery_percent = int(msg.battery * 10 / 16)
    battery_width = int(battery_percent / 100 * (img_width - 20))
    cv2.rectangle(img, (10, 10), (battery_width + 10, img_height - 10), (0, 255, 0), -1)
    cv2.putText(img, "Battery: {}%".format(battery_percent), (img_width//2-50, img_height//2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    if msg.battery < 140: #15% per
        rospy.logwarn("Now I4 battery status low: %s", msg.battery)
    else:
        rospy.loginfo("Now I4 battery status low:%s", msg.battery)
    cv2.imshow('Battery Status', img)
    cv2.waitKey(1)

rospy.init_node('battery_monitor')
rospy.Subscriber('/mobile_base/sensors/core', SensorState, battery_callback)
rospy.spin()










'''rostopic mobile_base/sensors/core 가 배터리 확인 하는 rostopic이고 퍼센트가 100/16으로 하면 1000%가 나오기 때문에 10/16으로 만듬.

battery_monitor.py 파일에 있던 내용 추가.
ref.
https://prlabhotelshoe.tistory.com/7 CV2.putText
https://copycoding.tistory.com/146 CV2.rectangel
obstacle_avoidance.py 에서 사용했던 kobuki_msgs.msg import sound 사용


import rospy
from kobuki_msgs.msg import SensorState

def battery_callback(msg):
    if msg.battery < 140: #15% per
        rospy.logwarn("Now I4 battery status low: %s", msg.battery)
    else:
        rospy.loginfo("Now I4 battery status low:%s", msg.battery)

rospy.init_node('battery_monitor')
rospy.Subscriber('/mobile_base/sensors/core', SensorState, battery_callback)
rospy.spin()

#base.yaml
# battery voltage at full charge (100%) (float, default: 16.5)
battery_capacity: 16.5

# battery voltage at first warning (15%) (float, default: 13.5)
battery_low: 14.0

# battery voltage at critical level (5%) (float, default: 13.2)
battery_dangerous: 13.2
'''

