import cv2
import numpy as np
import imutils
from pynput import keyboard
import torch
import rospy
from geometry_msgs.msg import Twist

KILL = False

# TrackingAPI
def tracking():

    ######## yolov5 ########
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

    # 클래스 리스트랑 색상 정보 가져오기
    classes = model.module.names if hasattr(model, 'module') else model.names
    colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in classes]
    ######## yolov5 ########

    ######## tracking ########
    trackers = [cv2.TrackerTLD_create, cv2.TrackerCSRT_create]
    trackerIdx = 0  # 트랙커 생성자 함수 선택 인덱스
    tracker = None
    isFirst = True

    fps = cap.get(cv2.CAP_PROP_FPS) # 프레임 수 구하기
    delay = int(1000/fps)
    win_name = 'Tracking APIs'
    ######## tracking ########


    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print('Cannot read video file')
            break

        ######## yolov5 ########
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

            # 사람 박스의 중심과 프레임 중심 간의 오차 계산하기 
            error_x = (x - w/2) / w

            # 오차 정보 이용 -> 코부기 제어하기 
            kobuki_twist = Twist()
            if (person_box[2]-person_box[0]) < 270.0 :
                kobuki_twist.linear.x = 0.5
            
            kobuki_twist.angular.z = -error_x * 1.3 # *2 
            kobuki_velocity_pub.publish(kobuki_twist)
        ######## yolov5 ########
        
        ######## tracking ########
        img_draw = frame.copy()
        if tracker is None: # 트랙커 생성 안된 경우
            cv2.putText(img_draw, "Press the Space to set ROI!!", \
                (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2,cv2.LINE_AA)
        else:
            ok, bbox = tracker.update(frame)   # 새로운 프레임에서 추적 위치 찾기 ---③
            (x,y,w,h) = bbox
            if ok: # 추적 성공
                cv2.rectangle(img_draw, (int(x), int(y)), (int(x + w), int(y + h)), \
                            (0,255,0), 2, 1)
            else : # 추적 실패
                cv2.putText(img_draw, "Tracking fail.", (100,80), \
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2,cv2.LINE_AA)
        trackerName = tracker.__class__.__name__
        cv2.putText(img_draw, str(trackerIdx) + ":"+trackerName , (100,20), \
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0),2,cv2.LINE_AA)

        cv2.imshow(win_name, img_draw)
        key = cv2.waitKey(delay) & 0xff
        # 스페이스 바 또는 비디오 파일 최초 실행 ---④
        if key == ord(' ') or (video_src != 0 and isFirst): 
            isFirst = False
            roi = cv2.selectROI(win_name, frame, False)  # 초기 객체 위치 설정
            if roi[2] and roi[3]:         # 위치 설정 값 있는 경우
                tracker = trackers[trackerIdx]()    #트랙커 객체 생성 ---⑤
                isInit = tracker.init(frame, roi)
        elif key in range(48, 56): # 0~7 숫자 입력   ---⑥
            trackerIdx = key-48     # 선택한 숫자로 트랙커 인덱스 수정
            if bbox is not None:
                tracker = trackers[trackerIdx]() # 선택한 숫자의 트랙커 객체 생성 ---⑦
                isInit = tracker.init(frame, bbox) # 이전 추적 위치로 추적 위치 초기화
        elif key == 27 : 
            break
        ######## tracking ########

    else:
        print( "Could not open video")
    out.write(frame)
    cv2.imshow('Webcam Tracking', frame)

# Yolov5
def yolov5():
    # 욜로 5
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

    # 클래스 리스트랑 색상 정보 가져오기
    classes = model.module.names if hasattr(model, 'module') else model.names
    colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in classes]

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

            # 사람 박스의 중심과 프레임 중심 간의 오차 계산하기 
            error_x = (x - w/2) / w

            # 오차 정보 이용 -> 코부기 제어하기 
            kobuki_twist = Twist()
            if (person_box[2]-person_box[0]) < 270.0 :
                kobuki_twist.linear.x = 0.5
            
            kobuki_twist.angular.z = -error_x * 1.3 # *2 
            kobuki_velocity_pub.publish(kobuki_twist)
            
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

    # 웹캠 초기화
    video_src = 0 # 비디오 파일과 카메라 선택 ---②
    cap = cv2.VideoCapture(video_src)
    
    # 첫 번째 프레임 가져오기
    ret, frame = cap.read()
    (h0, w0) = frame.shape[:2]

    # 비디오 라이터 설정하기
    fourcc = cv2.VideoWriter_fourcc('M','J','P','G') #코덱 설정 후 비디오 라이터 객체 생성하기
    out = cv2.VideoWriter('webcam_tracker.avi',fourcc, 20.0, (w0,h0),True)

    try:
        tracking()
        #yolov5()
    except rospy.ROSInterruptException:
        pass