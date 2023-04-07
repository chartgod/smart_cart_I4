# -*- coding: utf-8 -*-
#아이포_이승헌
# 5초 이상 객체 검출 되었을 때 그 객체만 따라갈 수 있게 수정
# 라이다를 이용해서 거리 간격을 받아와서 간격을 유지하면서 따라갈 수 있게 수정.

import cv2
import numpy as np
import imutils
from pynput import keyboard
import torch
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

KILL = False

# 종료 키
def on_press(key):
    if key.char == 'q':
        global KILL
        KILL = True

person_start_time = None
lidar_distance = None

# 라이다 스캔 데이터 처리 함수
def lidar_scan_callback(scan_data):
    global lidar_distance
    lidar_distance = min(scan_data.ranges)

def main():
    # 욜로 5
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

    # 클래스 리스트랑 색상 정보 가져오기
    classes = model.module.names if hasattr(model, 'module') else model.names
    colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in classes]

    # 웹캠 초기화
    cap = cv2.VideoCapture(0)

    # 첫 번째 프레임 가져오기
    ret, frame = cap.read()
    (h, w) = frame.shape[:2]

    # 비디오 라이터 설정하기
    fourcc = cv2.VideoWriter_fourcc('M','J','P','G') #코덱 설정 후 비디오 라이터 객체 생성하기
    out = cv2.VideoWriter('webcam_tracker.avi',fourcc, 20.0, (w,h),True)

    # 라이다 데이터 구독하기
    lidar_sub = rospy.Subscriber('/scan', LaserScan, lidar_scan_callback)

    while True:
        # 다음 프레임 가져오기 후 그레이스케일로 변환하기
        ret, frame = cap.read()

        # yolov5 모델에서 검출 결과 가져오기
        results = model(frame)

        # 검출된 객체 바운딩이랑 라벨 가져오기 
        boxes = results.xyxy[0].cpu().numpy()
        labels = results.xyxyn[0][:, -1].cpu().numpy()

        # 사람 객체 찾기
        person_boxes = []
        for i, label in enumerate(labels):
            if classes[int(label)] == 'person':
                person_boxes.append(boxes[i])
        
        # 사람 객체가 검출되면 
        if person_boxes:
            # 가장 큰 면적을 가진 박스 가져오기 
            person_box = max(person_boxes, key=lambda x: (x[2] - x[0]) * (x[3] - x[1]))
            #print(person_box)

            # 사람 박스 중심 좌표 가져오기 
            x, y = (int((person_box[0] + person_box[2]) / 2), int((person_box[1] + person_box[3]) / 2))
            # 프레임에 사람 박스, 중심점 그리기
            cv2.rectangle(frame, (int(person_box[0]), int(person_box[1])), (int(person_box[2]), int(person_box[3])), (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

            # 검출된 객체가 5초 이상 유지되는지 확인
            if person_start_time is None:
                person_start_time = time.time()
            elif time.time() - person_start_time >= 5:
                # 사람 박스의 중심과 프레임 중심 간의 오차 계산하기
                error_x = (x - w/2) / w

                # 오차 정보 이용 -> 코부기 제어하기
                kobuki_twist = Twist()

                if lidar_distance is not None:
                    if (person_box[2] - person_box[0]) < 270.0 and lidar_distance > 1.0:
                        kobuki_twist.linear.x = 0.3

                kobuki_twist.angular.z = -error_x * 1.3
                kobuki_velocity_pub.publish(kobuki_twist)
        else:
            # 사람이 검출되지 않으면 person_start_time을 다시 None으로 설정
            person_start_time = None

        # imshow하기
        out.write(frame)
        cv2.imshow('Webcam Tracking', frame)

        # 키 확인
        if KILL:
            print("\nFinished")
            out.release()
            cv2.destroyAllWindows()
            exit()
        cv2.waitKey(1)

if __name__ == '__main__':

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # ros노드 초기화
    rospy.init_node('webcam_tracker', anonymous=False)

    # pub 코부기 노드
    kobuki_velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    try:
        main()
    except rospy.ROSInterruptException:
        pass


