import cv2
import mediapipe as mp
import imutils
from imutils.video import VideoStream
from pyautogui import size as sz
import time

mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

vid = VideoStream(src=0, framerate=10).start()
time.sleep(2.0)

while True:
    frame = vid.read()

    if frame is None:
        break
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    frame = imutils.resize(frame, height=int(sz()[1] * 0.9))# , width=int(sz()[0]*0.9))
    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(image_rgb)

    if results.pose_landmarks:
        mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

    cv2.imshow("AthletiQ Pose Detection", frame)

pose.close()
vid.stop()
cv2.destroyAllWindows()
