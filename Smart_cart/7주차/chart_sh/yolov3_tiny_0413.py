#연희직업전문학교_아이포_ 이승헌 , 탁태우
#오픈소스 mobilessd -> yolov5_opencv 수정_이승헌 2023.04.03
#pid내용 추가 및 수정. _이승헌 2023.04.05
#kp,ki,kd 수정 _이승헌 pid내용 제거 04.08
# 5초간 객체 인식 하면 그 객체 따라가게 하기 _이승헌 04.07
# 라이다 거리 간격 받아오기. 0.5거리 _이승헌 04.07
# 4월10일 수정 - > rplidar 0.3 이내면 정지하기. -> 0.5간격으로 따라가기. _이승헌
# navigation.launch와 연동 시도 _0412_ 이승헌
# 변환행렬을 이용해서 객체 좌표 라이다 좌표계로 받아옴 _이승헌 0412
# 받아온 라이다 좌표계를 사용해서 로봇을 제어함 _ 이승헌 0412
#로봇과 사람 사이의 거리가 일정 간격(0.5m) 내에 있을 경우 로봇이 후진하지 않도록 변경 _이승헌
# 0413 해상도 조절 _ 이승헌
import roslaunch
import roslib
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
desired_distance = 0.4  # 40cm
distance_tolerance = 0.05  # 5cm

# 종료 키
def on_press(key):
    if key.char == 'q':
        global KILL
        KILL = True

person_start_time = None
lidar_distance = None

def lidar_callback(scan_data):
    global lidar_distance
    lidar_distance = min(scan_data.ranges)

#rospy.init_node('webcam_tracker', anonymous=False)

rospy.Subscriber('/scan', LaserScan, lidar_callback)
kobuki_velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# 외부 파라미터를 이용해 카메라에서 검출된 사람 객체의 좌표를 라이다 좌표계로 변환하는 함수
def transform_camera_to_lidar(x, y, z, T):
    camera_point = np.array([x, y, z, 1]).reshape(4, 1)
    lidar_point = np.dot(T, camera_point)
    return lidar_point[:3]

def main():
    # 욜로3-tiny
    model = torch.hub.load('ultralytics/yolov3-tiny', 'yolov3-tiny', pretrained=True)


    # 클래스 리스트랑 색상 정보 가져오기
    classes = model.module.names if hasattr(model, 'module') else model.names
    colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in classes]


    # 웹캠 초기화
    cap = cv2.VideoCapture(0)
    cap.set(cv2.PROP_FRAME_WIDTH, 640) #0413 320X240 도 사용함
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) #0413

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

        # 사람 박스 중심 좌표 가져오기 
            u, v = (int((person_box[0] + person_box[2]) / 2), int((person_box[1] + person_box[3]) / 2))
            # 3D 좌표로 변환
            # 카메라 캘리브레이션 값 대신 대략적인 값 사용
            cx = w / 2
            cy = h / 2
            fx = w / 2
            fy = h / 2
            z = lidar_distance
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
        
            # 카메라 좌표계에서 라이다 좌표게로 변환
            T = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, -0.05],
                          [0, 0, 0, 1]])
 
            lidar_point = transform_camera_to_lidar(x , y , z, T)
            # 프레임에 사람 박스, 중심점 그리기 
            cv2.rectangle(frame, (int(person_box[0]), int(person_box[1])), (int(person_box[2]), int(person_box[3])), (0, 255, 0), 2)
            cv2.circle(frame, (u, v), 5, (0, 0, 255), -1)

            # 검출된 객체가 5초 이상 유지되는지 확인
            if person_start_time is None:
                person_start_time = time.time()
            elif time.time() - person_start_time >= object_tracking_time:
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
            # 로봇과 사람 간격 계산하기
            robot_pos = (0, 0)  # 로봇 좌표
            person_pos = lidar_point[:2]  # 사람 좌표
            distance = np.linalg.norm(np.array(robot_pos) - np.array(person_pos))
            # 로봇과 사람 간의 거리가 일정 간격 이내일 경우
            if distance < desired_distance + 0.5:
                kobuki_twist.linear.x = 0.0  # 로봇 정지
                kobuki_twist.angular.z = 0.0

            # 로봇과 사람 간의 거리가 일정 간격 이상일 경우
            elif distance >= desired_distance + 0.5:
                # 로봇과 사람이 멀어지고 있을 경우
                if distance > lidar_distance + 0.5:
                    kobuki_twist.linear.x = 0.3  # 로봇 전진
                    kobuki_twist.angular.z = 0.0

                # 로봇과 사람이 가까이 다가가고 있을 경우
                else:
                    kobuki_twist.linear.x = -0.1  # 로봇 후진
                # 로봇이 사람을 향하도록 방향 조절하기
                    if person_pos[0] > robot_pos[0]:
                        kobuki_twist.angular.z = 0.3  # 로봇 좌회전
                    elif person_pos[0] < robot_pos[0]:
                        kobuki_twist.angular.z = -0.3  # 로봇 우회전
                    else:
                        kobuki_twist.angular.z = 0.0

            # 로봇 제어 메시지 발행
            kobuki_velocity_pub.publish(kobuki_twist)


    # 검출된 객체가 없을 경우 
        else:
        # 로봇을 정지시키기
            kobuki_twist = Twist()
            kobuki_velocity_pub.publish(kobuki_twist)

        # 프레임에 시간 정보 추가하기
        cv2.putText(frame, time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 프레임 출력하기 
        cv2.imshow('webcam_tracker', frame)

        # 비디오 라이터에 프레임 추가하기
        out.write(frame)

        # q 키를 누르면 종료하기
        if rospy.is_shutdown():
            break

# 웹캠과 비디오 라이터 객체 해제하기
    cap.release()
    out.release()

    # 모든 윈도우 창 닫기
    cv2.destroyAllWindows()
if __name__ == '__main__':
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ['kobuki_node/kobuki_navigation.launch'])
    launch.start()

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

