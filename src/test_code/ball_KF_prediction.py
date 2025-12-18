import numpy as np
from numpy.linalg import inv
import imutils
from imutils.video import VideoStream
from pyautogui import size as sz
import cv2
import time


def KalmanFilter(mu_prev, sigma_prev, z):
    mu_bar = A_t.dot(mu_prev)
    sigma_bar = A_t.dot(sigma_prev).dot(A_t.transpose()) + R_t
    if z is None:
        return mu_bar, sigma_bar
    else:
        K_t = sigma_bar.dot(C_t.transpose()).dot(inv(C_t.dot(sigma_bar).dot(C_t.transpose()) + Q_t))
        mu = mu_bar + K_t.dot(z - C_t.dot(mu_bar))
        sigma = (np.identity(2) - K_t.dot(C_t)).dot(sigma_bar)
        return mu, sigma


A_t = np.array([[1, 1], [0, 1]])
G = np.array([[0.5], [1]])
R_t = G.dot(G.transpose())
C_t = np.array([[1, 0]])
Q_t = np.array([[1]])
mu_t = np.array([[0, 0], [0, 0]])
sigma_t = np.array([[0, 0], [0, 0]])

orangeLower = (10, 150, 100)
orangeUpper = (40, 255, 255)
found = False

vid = VideoStream(src=0, framerate=10).start()
time.sleep(2.0)

while True:
    curr_t = time.time()
    frame = vid.read()

    if frame is None:
        break
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    frame = imutils.resize(frame, height=int(sz()[1]*0.9))#, width=int(sz()[0]*0.9))
    blr = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, orangeLower, orangeUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cntr = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cntr = imutils.grab_contours(cntr)

    if len(cntr) > 0:
        found = True
        c = max(cntr, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
        cv2.circle(frame, center, 5, (0, 255, 0), -1)

    if found and (len(cntr) > 0):
        mu_t, sigma_t = KalmanFilter(mu_t, sigma_t, np.array([[x, y]]))
        x_bel, y_bel = mu_t[0][0], mu_t[0][1]
        cv2.circle(frame, (int(x_bel), int(y_bel)), int(radius), (255, 0, 0), 2)
        cv2.circle(frame, (int(x_bel), int(y_bel)), 5, (255, 0, 0), -1)
    elif found and (len(cntr) <= 0):
        mu_t, sigma_t = KalmanFilter(mu_t, sigma_t, None)
        x_bel, y_bel = mu_t[0][0], mu_t[0][1]
        cv2.circle(frame, (int(x_bel), int(y_bel)), int(radius), (255, 0, 0), 2)
        cv2.circle(frame, (int(x_bel), int(y_bel)), 5, (255, 0, 0), -1)

    cv2.imshow("AthletiQ KF Tracking", frame)
    time.sleep(max(1. / 25 - (time.time() - curr_t), 0))

vid.stop()
cv2.destroyAllWindows()
