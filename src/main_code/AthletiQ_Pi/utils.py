import os
import sys
import logging
import builtins
import socket
import av
import io
import json
import serial
import time
from datetime import datetime
from dotenv import load_dotenv
import cv2
import imutils
import numpy as np
from numpy.linalg import inv
import mediapipe as mp
from collections import deque
import firebase_admin
from firebase_admin import credentials, storage


load_dotenv()
USERID_FILE_PATH = os.getenv("USERID_FILE_PATH")
LOG_FILE_PATH = os.getenv("LOG_FILE_PATH")
FIREBASE_CRED_PATH = os.getenv('FIREBASE_CRED_PATH')
FIREBASE_STORAGE_BUCKET = os.getenv('FIREBASE_STORAGE_BUCKET')

computation_rate: int = 25
fps: int = 10
res: tuple[int, int] = (1280, 720)

port: str = '/dev/serial0'
baud: int = 230400

min_det_conf: float = 0.5
min_tra_conf: float = 0.5
width_cutoff: float = 0.3

states: tuple[str, str, str] = ("Prep", "Set", "Thrown")
modes: tuple[str, str] = ("Side", "Front")
mode_idx: int = 0

past_t: int = 2
future_t: int = 6
final_t: int = 3
snap_frame_diff: int = 3
duration_thresh: float = 1.0

save_files: bool = False
upload_files: bool = True


def init_logger(log_file_path):
    def config_logger():
        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s - %(levelname)s - %(message)s",
            handlers=[
                logging.FileHandler(log_file_path),
                logging.StreamHandler()
            ]
        )

    def log_print(*args, **kwargs):
        sep = kwargs.get("sep", " ")
        end = kwargs.get("end", "\n")

        message = sep.join(str(arg) for arg in args) + end
        if message.strip():
            logging.info(message)

    def handle_exception(exc_type, exc_value, exc_traceback):
        if issubclass(exc_type, KeyboardInterrupt):
            sys.__excepthook__(exc_type, exc_value, exc_traceback)
            return

        logging.error(
            "UNCAUGHT EXCEPTION",
            exc_info=(exc_type, exc_value, exc_traceback)
        )

    config_logger()

    logging.info("=" * 80)
    logging.info("NEW RUN STARTED")
    logging.info(f"Time : {datetime.now().isoformat()}")
    logging.info(f"PID  : {os.getpid()}")
    logging.info("=" * 80)

    builtins.print = log_print
    sys.excepthook = handle_exception


def calc_angle(p1, p2, p3):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3

    A = np.array([x1, y1])
    B = np.array([x2, y2])
    C = np.array([x3, y3])

    BA = A - B
    BC = C - B

    cos_angle = np.dot(BA, BC) / (np.linalg.norm(BA) * np.linalg.norm(BC))
    angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
    angle_deg = np.degrees(angle)

    if angle_deg > 180:
        angle_deg = 360 - angle_deg

    return angle_deg


def calc_midpoint(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    midpoint = ((x1 + x2) // 2, (y1 + y2) // 2)

    return midpoint


def draw_circles(frame, center, center_hat, radius):
    if radius:
        cv2.circle(frame, center, int(radius), (0, 255, 0), 2)
        cv2.circle(frame, center, 5, (0, 255, 0), -1)
        cv2.circle(frame, center_hat, int(radius), (255, 0, 0), 2)
        cv2.circle(frame, center_hat, 5, (255, 0, 0), -1)

    return frame


def draw_landmarks(frame, pd, results, coords):
    if results.pose_landmarks:
        pd.mp_drawing.draw_landmarks(frame, results.pose_landmarks, pd.mp_pose.POSE_CONNECTIONS)
    if coords:
        _, _, _, cmass = coords
        cv2.circle(frame, cmass, 15, (255, 0, 255), -1)

    return frame


def draw_text(frame, s_t, mode):
    if s_t is None or s_t < 0:
        state_txt = "?"
    else:
        state_txt = states[s_t]

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    color = (0, 255, 0)
    thickness = 2
    line_type = cv2.LINE_AA

    cv2.putText(frame, state_txt, (50, 50), font, font_scale, color, thickness, line_type)
    cv2.putText(frame, mode, (50, 100), font, font_scale, color, thickness, line_type)

    return frame


def draw_rec(frame, is_rec):
    if not is_rec:
        return frame

    txt = "REC"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    color = (0, 0, 255)
    thickness = 2
    line_type = cv2.LINE_AA

    _, w, _ = frame.shape
    x = w - 100
    y = 50
    (txt_w, txt_h), _ = cv2.getTextSize(txt, font, font_scale, thickness)

    cv2.circle(frame, (x-int(txt_w*0.4), y-txt_h//2), int(txt_h*0.45), color, -1)
    cv2.putText(frame, txt, (x, y), font, font_scale, color, thickness, line_type)

    return frame
