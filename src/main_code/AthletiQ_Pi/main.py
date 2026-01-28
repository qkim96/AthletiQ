#!/usr/bin/env python3

"""
This framework implements a real-time basketball action detection and pose analysis pipeline
using probabilistic object localization/tracking algorithm and Bayesian state estimation model.

A Kalman filter is used to recursively localize and track the basketball from a camera live
feed, combining motion priors with noisy visual measurements to remain robust to occlusions
and missed detections. Player pose landmarks are extracted using MediaPipe and fused with the
estimated position of the ball to construct binary observation features for action analysis.

Shooting action recognition is formulated as a Hidden Markov Model with three hidden action
states: 'Prep' (Π), 'Set' (Σ), and 'Thrown' (Τ). Having binary observations and factorized
features, the system uses a Bernoulli HMM with factorized emissions, and at each time step
(camera frame), online Bayesian filtering is performed via the HMM forward algorithm with
a local Maximum a Posteriori (MAP) deterministic decision rule to infer the hidden states.
When the filtered posterior exhibits low confidence, a 1-step Viterbi-style local decoding
without backtracking is applied on the previously selected state to improve the consistency.

Camera orientation (front/side) is determined by data received via UART serial communication
from an Arduino and conditions the observation model. The system also sends UART commands to
rotate the camera when the player approaches the frame boundaries. Upon detecting a shooting
action, the system automatically records a video clip, combines past frames using a buffer,
analyzes the shooting pose, serializes results to JSON, and transmits H.264-formatted video
clip and JSON file containing the analyzed pose data to a server for web-based visualization.
"""


import utils
from utils import *
from camera_source import CameraSource
from object_localization import ObjectLocalizer
from pose_detection import PoseDetector
from state_estimation import StateEstimator
from video_encoding import VideoEncoder
from data_extraction import DataExtractor
from data_serialization import DataSerializer
from server_publication import ServerPublisher
from serial_communication import SerialCommunicator


def run():
    pub = ServerPublisher(firebase_cred_path=FIREBASE_CRED_PATH, firebase_storage_bucket=FIREBASE_STORAGE_BUCKET)
    SC.comm_init(port, baud)

    cam = CameraSource(platform="pi", src=0, rate=fps, size=res)
    cam.initialize()
    cam.start()

    time.sleep(2.0)

    while True:
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=1)
            print("WiFi Connected")
            break
        except OSError:
            print("Waiting for WiFi...")
            time.sleep(1.0)

    SC.transmit_uart(sync_start=True)
    print("Start byte sent")

    while True:
        rxByte = SC.receive_uart()

        if rxByte == b'\xAA':
            print("Received ACK byte")
            break

        print("Waiting for ACK byte...")
        time.sleep(1.0)

    while True:
        loop_start_t = time.time()
        frame = cam.read()

        if frame is None:
            logging.warning("No frame received from camera")
            break
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        frame = cv2.flip(frame, 1)

        c_measured, c_posteriori, r = OL.process_frame(frame)
        results, coords = PD.process_frame(frame)

        SE.observe(c_posteriori, r, coords)
        SE.estimate()

        filetime = datetime.now().strftime('%Y%m%d_%H%M%S')
        clip_ready = False
        clip_success = False
        json_success = False

        h264_bytes, timestamps = VE.update_buffer(frame, SE.s_t, SE.s_prev)

        if h264_bytes:
            clip_ready = True
            clip_filename = f'{filetime}.h264'

            if upload_files:
                clip_success = \
                    pub.upload_file(
                        data_bytes=h264_bytes,
                        remote_path=f'{filetime}/{clip_filename}',
                        content_type='video/h264'
                    )

            if save_files:
                with open(clip_filename, "wb") as f:
                    f.write(h264_bytes)
                print(f"Saved clip: {clip_filename}")

        if time.time() - VE.finalized_time >= past_t:
            data_ready =\
                DE.extract_data(
                    frame,
                    clip_ready,
                    VE.is_recording,
                    VE.is_in_action,
                    timestamps,
                    utils.mode_idx,
                    c_posteriori,
                    results
                )
        else:
            data_ready = False

        if data_ready:
            dict_data = DS.to_dict(DE)
            json_data = DS.to_json(dict_data)
            json_filename = f'{filetime}.json'
            DE.clear_data()

            if upload_files:
                json_success = \
                    pub.upload_file(
                        data_bytes=json_data,
                        remote_path=f'{filetime}/{json_filename}',
                        content_type='application/json'
                    )

            if save_files:
                with open(json_filename, "w", encoding="utf-8") as f:
                    f.write(json_data)
                print(f"Saved json: {json_filename}")

        if clip_success and json_success:
            print(f"Successfully uploaded files {clip_filename} and {json_filename} to Firebase Storage")

        rotateL, rotateR = PD.body_position_LR(frame, coords)
        tx_data = SC.transmit_uart(rotateL, rotateR)
        rx_data = SC.receive_uart()

        if tx_data:
            print("UART Data Transmitted: ", tx_data)
        if rx_data and not VE.is_recording:
            utils.mode_idx = int(rx_data)

        frame_view = frame.copy()
        frame_view = draw_circles(frame_view, c_measured, c_posteriori, r)
        frame_view = draw_landmarks(frame_view, PD, results, coords)
        frame_view = draw_text(frame_view, SE.s_t, modes[utils.mode_idx])
        frame_view = draw_rec(frame_view, VE.is_recording)

        cv2.imshow("AthletiQ", frame_view)
        time.sleep(max((1. / computation_rate) - (time.time() - loop_start_t), 0))

    PD.release()
    cam.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    init_logger(LOG_FILE_PATH)

    OL = ObjectLocalizer()
    PD = PoseDetector()
    SE = StateEstimator()
    VE = VideoEncoder()
    DE = DataExtractor()
    DS = DataSerializer()
    SC = SerialCommunicator()

    run()
