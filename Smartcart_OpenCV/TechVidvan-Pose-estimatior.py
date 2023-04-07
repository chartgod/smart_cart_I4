# TechVidvan Human pose estimator
# import necessary packages

import cv2
import mediapipe as mp

# initialize Pose estimator
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

pose = mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5)
#youtube
# import pafy
# url = 'https://www.youtube.com/watch?v=u_Q7Dkl7AIk'
# video = pafy.new(url)
# best = video.getbest()
#cap = cv2.VideoCapture(best.url)

# create capture object
#cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture('./vtest.avi')


while cap.isOpened():
    # read frame from capture object
    _, frame = cap.read()

    try:
        # convert the frame to RGB format
        RGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # process the RGB frame to get the result
        results = pose.process(RGB)

        print(results.pose_landmarks)
        # draw detected skeleton on the frame
        mp_drawing.draw_landmarks(
            frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        # show the final output
        cv2.imshow('Output', frame)
    except:
        break
    if cv2.waitKey(1) == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
