# 1014.py
# pip install opencv-contrib-python 

'''
In case of  GOTURN,
you need the pretrained caffe model:'goturn.prototxt','goturn.caffemodel'
in
ref1: https://github.com/Mogball/goturn-files
ref2: https://github.com/Auron-X/GOTURN_Training_Toolkit
'''

import cv2
import numpy as np

#1
roi  = None
drag_start = None
mouse_status = 0
tracking_start = False
def onMouse(event, x, y, flags, param = None):
     global roi
     global drag_start
     global mouse_status
     global tracking_start   
     if event == cv2.EVENT_LBUTTONDOWN:
          drag_start = (x, y)
          mouse_status = 1
          tracking_start = False
     elif event == cv2.EVENT_MOUSEMOVE:
          if flags == cv2.EVENT_FLAG_LBUTTON:
               xmin = min(x, drag_start[0])
               ymin = min(y, drag_start[1])
               xmax = max(x, drag_start[0])
               ymax = max(y, drag_start[1])
               roi = (xmin, ymin, xmax, ymax)
               mouse_status = 2 # dragging
     elif event == cv2.EVENT_LBUTTONUP:
          mouse_status = 3       # complete

#2
def createTracker(track_type=0):
     if track_type == 0:
          tracker = cv2.TrackerCSRT_create()
     elif track_type == 1:
          tracker = cv2.TrackerKCF_create()
     elif track_type == 2:
          tracker = cv2.TrackerMIL_create()
     else:
          tracker = cv2.TrackerGOTURN_create()   
     return tracker

#3          
cv2.namedWindow('tracking')
cv2.setMouseCallback('tracking', onMouse)

#cap = cv2.VideoCapture('http://192.168.123.1:4747/mjpegfeed') # DroidCam 앱 사용 droid cam
#cap = cv2.VideoCapture('./data/ball.wmv')
cap = cv2.VideoCapture(0)
if (not cap.isOpened()): 
     print('Error opening video')    
height, width = (int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                 int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
tracker = None
  
#4 
while True:
     ret, frame = cap.read()
     if not ret: break

     if mouse_status == 2:
          x1, y1, x2, y2 = roi
          cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)  
     if mouse_status == 3:
          print('initialize....')
          mouse_status = 0
          track_box = (roi[0], roi[1], roi[2]-roi[0], roi[3]-roi[1])
          if tracker != None: #이미 트렉커가 있으면 삭제
               del tracker

          tracker = createTracker() #트렉커 생성
          tracker.init(frame, track_box) #객체 초기화 track_box:바운딩박스
          tracking_start = True
      
     if tracking_start:
          ret, track_box = tracker.update(frame) #바운딩 박스 갱신
          if ret:  # Tracking success
              x, y , w, h =  track_box
              p1 = (int(x), int(y))
              p2 = (int(x + w), int(y + h))
              cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
                      
     cv2.imshow('tracking', frame)
     key = cv2.waitKey(200)
     if key == 27:
          break
if cap.isOpened():
    cap.release();
cv2.destroyAllWindows()
