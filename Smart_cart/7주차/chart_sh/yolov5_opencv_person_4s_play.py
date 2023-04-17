
#연희직업전문학교_아이포_ 이승헌 , 탁태우
#아이디어, 코드작성 이승헌 최종 수정 04.15
#오픈소스 mobilessd -> yolov5_opencv 수정_이승헌 2023.04.03
#if (person_box[2]-person_box[0]) < 270.0 : 테스트 검토, 이승헌, 탁태우
#pid내용 추가 및 수정. _이승헌 2023.04.05
#opencv로 프레임 오차 및 사람 중심간 오차 계산 _이승헌
#kp,ki,kd 수정 _이승헌 pid내용 제거 04.08
# 5초간 객체 인식 하면 그 객체 따라가게 하기 _이승헌 04.07
# 라이다 거리 간격 받아오기. 0.5거리 _이승헌 04.07
# 4월10일 수정 - > rplidar 0.3 이내면 정지하기. -> 0.5간격으로 따라가기. _이승헌
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
desired_distance = 0.5
distance_tolerance = 0.1

# 종료 키
def on_press(key):
    if key.char == 'q':
        global KILL
        KILL = True

person_start_time = None
lidar_distance = None

def lidar_callback(scan_data):
    global lidar_distance
    ranges = np.array(scan_data.ranges) #lidar_distance = min(scan_data.ranges)
    # 0~180도 범위 수정하기
    ranges = ranges[:180]
    lidar_distance = np.min(ranges)

rospy.init_node('webcam_tracker', anonymous=False)

rospy.Subscriber('/scan', LaserScan, lidar_callback)

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

    # 객체 검출 유지 시간
    object_tracking_time = 4  # 4초 유지

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
        kobuki_twist = Twist()
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
            elif time.time() - person_start_time >=5: #time.time() - person_start_time >= object_tracking_time:
                # 사람 박스의 중심과 프레임 중심 간의 오차 계산하기 
                error_x = (x - w/2) / w

                # 오차 정보 이용 -> 코부기 제어하기 
                if (person_box[2]-person_box[0]) < 270.0 :
                    kobuki_twist.linear.x = 0.4

                if person_start_time is not None and time.time() - person_start_time >= object_tracking_time:
                    error_x = (x - w/2) / w
                    kobuki_twist.angular.z = -error_x * 1.3 # *2

                if lidar_distance is not None:
                    distance_error = lidar_distance - desired_distance
                    if abs(distance_error) > distance_tolerance:
                        kobuki_twist.linear.x = 0.3 * np.sign(distance_error)
                    elif lidar_distance <= 0.3:
                        kobuki_twist.linear.x = 0.0 
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
