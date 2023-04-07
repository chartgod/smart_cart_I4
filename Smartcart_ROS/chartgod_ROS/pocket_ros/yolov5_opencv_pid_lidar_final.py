#연희직업전문학교_아이포_ 이승헌 , 탁태우
#오픈소스 mobilessd -> yolov5_opencv 수정_이승헌 2023.04.03
#pid내용 추가 및 수정. _이승헌 2023.04.05
#kp,ki,kd 수정 _이승헌
# 5초간 객체 인식 하면 그 객체 따라가게 하기 _이승헌 04.07
# 라이다 거리 간격 받아오기. 0.4거리 _이승헌 04.07

import cv2
import numpy as np
import imutils
from pynput import keyboard
import torch
import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

VELOCITY_WINDOW_SIZE = 10
KILL = False
# PID 제어 변수 초기화
prev_error = 0
integral = 0
velocity_x_list = []
# 라이다 거리 변수 초기화
distance = 0.0
DETECTION_DURATION = 5  # 초 단위

# ROS 노드 초기화
rospy.init_node('webcam_tracker')

# Kobuki 로봇의 속도 제어를 위한 ROS publisher 초기화
kobuki_velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

# 라이다 거리를 가져오기 위한 ROS subscriber 초기화
def scan_callback(data):
    global distance
    distance = data.ranges[0]

lidar_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)

# -KP: 0.1~1.0 / KI : 0.001~0.1 / KD: 0.01~0.5  일반적이다.
# PID 게인값 설정
KP = 0.4
KI = 0.01
KD = 0.05

# 종료 키
def on_press(key):
    if  key.char == 'q':
        global KILL
        KILL = False

def main():
    # 욜로 5

    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

    # 클래스 리스트랑 색상 정보 가져오기
    classes = model.module.names if hasattr(model, 'module') else model.names
    colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in classes]

    # 웹캠 초기화
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Failed to open webcam.")
        cap.release()
        cv2.destroyAllWindows()
        exit()
    # 글로벌 kill 강제 종료 되지 않게 하
        
    global KILL
    # person_id 및 first_detection_time 변수 초기화
    person_id = None
    first_detection_time = None
    # 이전 프레임의 위치 정보 초기화
    prev_x = None

    # 첫 번째 프레임 가져오기
    ret, frame = cap.read()
    if not ret: 
        print("Failed to read from webcam.")
        cap.release()
        cv2.destroyAllWindows()
        exit()
    (h, w) = frame.shape[:2]

    # 비디오 라이터 설정하기
    fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
    out = cv2.VideoWriter('webcam_tracker.avi',fourcc, 20.0, (w,h))

    # PID 제어 변수 초기화
    prev_error = 0
    integral = 0
    time_prev = time.time()
    velocity_x_list = []

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
            max_person_box = max(person_boxes, key=lambda x: (x[2] - x[0]) * (x[3] - x[1]))

    # 현재 시간 확인
            current_time = time.time()

    # 최근 검출된 객체가 없거나, 검출된 객체가 5초 이상 지났다면 새로운 객체 저장
            if person_id is None or current_time - first_detection_time > DETECTION_DURATION:
                person_id = max_person_box
                first_detection_time = current_time

    # 저장된 객체를 사용하여 중심 좌표 계산
            x, y = (int((person_id[0] + person_id[2]) / 2), int((person_id[1] + person_id[3]) / 2))

            #person_box = max(person_boxes, key=lambda x: (x[2] - x[0]) * (x[3] - x[1]))

            # 사람 박스 중심 좌표 가져오기 
            x, y = (int((max_person_box[0] + max_person_box[2]) / 2), int((max_person_box[1] + max_person_box[3]) / 2))

            # 프레임에 사람 박스, 중심점 그리기 
            cv2.rectangle(frame, (int(max_person_box[0]), int(max_person_box[1])), (int(max_person_box[2]), int(max_person_box[3])), (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

            # 라이다 거리가 0.4 이하인지 확인하기 
            if distance < 0.4:
                # 사람 박스의 중심과 프레임 중심 간의 오차 계산하기 
                error_x = (x - w/2) / w
        
                # PID 제어 계산하기
                integral += error_x * dt
                derivative = (error_x - prev_error) / dt if dt > 0 else 0
                output = KP * error_x + KI * integral + KD * derivative
        
                # 이전 오차 정보 저장하기
                prev_error = error_x

                # 이전 프레임과 현재 프레임의 시간 차 계산하기 
                time_now = time.time()
                dt = time_now - time_prev
                # 현재 프레임의 속도 계산하기 
                if dt > 0:
                    velocity_x = error_x / dt
                    velocity_x_list.append(velocity_x)
                else: 
                    velocity_x_list.append(0.0) # dt가 0일 때는 velocity_x가 nan이 되므로 0.0을 추가
                if len(velocity_x_list) > VELOCITY_WINDOW_SIZE:
                    velocity_x_list.pop(0)
                velocity_x_avg = np.mean(velocity_x_list)

                # 오차 정보 이용 -> 코부기 제어하기 
                kobuki_twist = Twist()
                kobuki_twist.linear.x = 0.5
                kobuki_twist.angular.z = -velocity_x_avg * 1.3  #1.3
                kobuki_velocity_pub.publish(kobuki_twist)
                print("평균 속도:", velocity_x_avg)
            else:
                kobuki_twist.linear.x = 0
                kobuki_twist.angular.z = 0
                kobuki_velocity_pub.publish(kobuki_twist)
        else:
            # 사람 객체가 검출되지 않으면 정지 명령 보내기 
            kobuki_twist = Twist()
            kobuki_twist.linear.x = 0
            kobuki_twist.angular.z = 0
            kobuki_velocity_pub.publish(kobuki_twist)

        # imshow하기
        out.write(frame)

        # 키 확인 
        if KILL:
            print("\nFinished")
            break
# 루프 종료 후 리소스 해제
    out.release()
    cap.release() #웹캠 리소스 해제 추가
    cv2.destroyAllWindows()
if __name__ == '__main__':
    
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    
    # ros노드 초기화
    rospy.init_node('webcam_tracker', anonymous=False)
    # 전역 변수 사용을 위해 global 키워드 사용하기
    
    # KILL 변수 초기화 하기 
    KILL = False

    # PID 제어 변수 초기화 
    
    # pub 코부기 노드 
    kobuki_velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
    try:
        main()
    except rospy.ROSInterruptException:
        pass                
