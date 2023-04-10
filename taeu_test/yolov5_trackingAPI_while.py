import cv2
import numpy as np
import imutils
from pynput import keyboard
import torch
import rospy
from geometry_msgs.msg import Twist

KILL = False

# 종료 키
def on_press(key):
    if key.char == 'q':
        global KILL
        KILL = True

# TrackingAPI
def tracking():
    
    ######## yolov5 ########
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

    # 클래스 리스트랑 색상 정보 가져오기
    classes = model.module.names if hasattr(model, 'module') else model.names
    colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in classes]
    ######## yolov5 ########

    ######## tracking ########
    trackers = [cv2.TrackerCSRT_create, cv2.TrackerTLD_create]
    trackerIdx = 0  # 트랙커 생성자 함수 선택 인덱스
    tracker = None
    isFirst = True

    fps = cap.get(cv2.CAP_PROP_FPS) # 프레임 수 구하기
    delay = int(1000/fps)
    win_name = 'Tracking APIs'
    ######## tracking ########
    ######## test ########
    ret, frame = cap.read()
    (h1, w1) = frame.shape[:2]
    print(h1,w1)
    select = 1
    x2 = 0
    y2 = 0
    w2 = 0
    h2 = 0
    while (select):
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
            person_box0 = max(person_boxes, key=lambda x: (x[2] - x[0]) * (x[3] - x[1]))
            # if h1>person_box0[2]:
            roi = (int(person_box0[0]),int(person_box0[1]),int(person_box0[2]-person_box0[0]),int(person_box0[3]-person_box0[1]))  # 초기 객체 위치 설정
            # print("roi:",roi)
            break
    ######## test ########

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print('Cannot read video file')
            break

        
        ######## tracking ########
        img_draw = frame.copy()
        if tracker is None: # 트랙커 생성 안된 경우
            cv2.putText(img_draw, "Press the Space to set ROI!!", \
                (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2,cv2.LINE_AA)
        else:
            ok, bbox = tracker.update(frame)   # 새로운 프레임에서 추적 위치 찾기 ---③
            (x2,y2,w2,h2) = bbox # 추적 박스 좌표
            if ok: # 추적 성공
                cv2.rectangle(img_draw, (int(x2), int(y2)), (int(x2 + w2), int(y2 + h2)), \
                            (0,255,255), 2, 1)
            else : # 추적 실패
                cv2.putText(img_draw, "Tracking fail.", (100,80), \
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2,cv2.LINE_AA)
        trackerName = tracker.__class__.__name__
        cv2.putText(img_draw, str(trackerIdx) + ":"+trackerName , (100,20), \
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0),2,cv2.LINE_AA)

        # cv2.imshow(win_name, img_draw)
        key = cv2.waitKey(delay) & 0xff
        
        # if roi[0] >= 0 and roi[1] >= 0 and roi[0]+roi[2] < frame.shape[1] and roi[1]+roi[3] < frame.shape[0]:
        #     isInit = tracker.init(frame, roi)
        # else:
        #     print("roi", roi)

        # 스페이스 바 또는 비디오 파일 최초 실행 ---④
        if select : # key == ord(' ') or (video_src != 0 and isFirst): 
            #isFirst = False
            select = 0
            # roi = (106, 191, 560, 478)
            # roi = cv2.selectROI(win_name, frame, False)  # 초기 객체 위치 설정
            # print("roi:",roi)
            if roi[2] and roi[3]:         # 위치 설정 값 있는 경우
                #print(roi[0],roi[1],roi[2],roi[3]) # ROI 박스 값
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
            x1, y1 = (int((person_box[0] + person_box[2]) / 2), int((person_box[1] + person_box[3]) / 2))

            # 프레임에 사람 박스, 중심점 그리기 
            cv2.rectangle(img_draw, (int(person_box[0]), int(person_box[1])), (int(person_box[2]), int(person_box[3])), (0, 255, 0), 2) #frame
            cv2.circle(img_draw, (x1, y1), 5, (0, 0, 255), -1) #frame
            
            # 사람 박스의 중심과 프레임 중심 간의 오차 계산하기 
            error_x = (x1 - w1/2) / w1
            
            padding = 50
            # 오차 정보 이용 -> 코부기 제어하기 # (x2,y2,w2,h2) = bbox 
            if (person_box[0]-padding) < x2 and (person_box[2]+padding) > (x2+w2) :
                if (person_box[1]-padding) < y2 and (person_box[3]+padding) > (y2+h2) :
                    kobuki_twist = Twist()
                    if (person_box[2]-person_box[0]) < 270.0 :
                        kobuki_twist.linear.x = 0.5
            
                    kobuki_twist.angular.z = -error_x * 1.3 # *2 
                    kobuki_velocity_pub.publish(kobuki_twist)
                    print("gogogo")
        ######## yolov5 ########

        cv2.imshow(win_name, img_draw)
        

    else:
        print( "Could not open video")
    out.write(frame)
    cv2.imshow('Webcam Tracking', frame)


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
    (h1, w1) = frame.shape[:2]

    # 비디오 라이터 설정하기
    fourcc = cv2.VideoWriter_fourcc('M','J','P','G') #코덱 설정 후 비디오 라이터 객체 생성하기
    out = cv2.VideoWriter('webcam_tracker.avi',fourcc, 20.0, (w1,h1),True)

    try:
        tracking()
        #yolov5()
    except rospy.ROSInterruptException:
        pass






        ######## test ########        
        # if key == ord(' ') or (video_src != 0 and isFirst): 
        #     isFirst = False
        #     # roi = cv2.selectROI(win_name, frame, False)  # 초기 객체 위치 설정
        #     # person_box0_x=int(person_box0[0])
        #     roi = (int(person_box0[0]),int(person_box0[1]),int(person_box0[2]),int(person_box0[3]))  # 초기 객체 위치 설정
        #     # print("roi:",roi) #roi: (190, 107, 348, 347)
        #     # print("person_box0:",person_box0) #person_box0: [     127.63      67.294      625.29         480     0.89085           0]
        #     # print("roi:",type(roi)) #roi: <class 'tuple'>
        #     # print("person_box0:",type(person_box0)) #person_box0: <class 'numpy.ndarray'>
        #     if roi[2] and roi[3]:         # 위치 설정 값 있는 경우
        #         #print(roi[0],roi[1],roi[2],roi[3]) # ROI 박스 값
        #         tracker = trackers[trackerIdx]()    #트랙커 객체 생성 ---⑤
        #         isInit = tracker.init(frame, roi)
        # elif key in range(48, 56): # 0~7 숫자 입력   ---⑥
        #     trackerIdx = key-48     # 선택한 숫자로 트랙커 인덱스 수정
        #     if bbox is not None:
        #         tracker = trackers[trackerIdx]() # 선택한 숫자의 트랙커 객체 생성 ---⑦
        #         isInit = tracker.init(frame, bbox) # 이전 추적 위치로 추적 위치 초기화
        # elif key == 27 : 
        #     break
        ######## test ########

        ######## test ########
        # person_box[0],person_box[1] / person_box[2],person_box[3] # 사람박스 좌표
        # x2,y2 / x2+w2,y2+h2 # 추적기 박스 좌표
        # if person_box[0] <  x2: #사람박스 첫번째 point x가 추적박스 x2 보다 작을 경우
        #     if person_box[2] > x2:

        ######## test ########