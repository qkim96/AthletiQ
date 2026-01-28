from utils import *


class DataExtractor:
    """
    Extracts and stores analyzed data from video frames.

    Attributes
    ----------
    frame_w : int
        Width of the video frame in pixels.
    frame_h : int
        Height of the video frame in pixels.
    mp_results_buffer : deque
        Buffer of MediaPipe pose results to compute and extract data from the past frames.
    id : str
        Six-digit user ID of the person to be recorded.
    timestamps : list of str
        Video timestamps [videoStart, shotTime, videoEnd] in YYYY-MM-DD HH:MM:SS format.
    mode : str
        Current mode while being recorded, "Side" or "Front".
    trajectory : list of [x, y] or None
        Ball trajectory coordinates in pixels (side mode only).
    angles : list of float or None
        [elbow, wrist] angles in degrees (side mode only).
    height : int
        Player height in pixels within the frame.
    h_loc : float or None
        Horizontal ball position within shoulder width with -1 being left and 1 being right (front mode only).
    v_loc : float
        Vertical ball position normalized by player height.
    ready_* : bool
        Flags indicating whether each data component is ready for serialization.
    """

    def __init__(self):
        """
        Initializes the DataExtractor instance.

        Sets up frame dimensions, internal buffers and initializes all data attributes to default values.
        All "ready" flags are set to False.
        """

        self.frame_w = 0
        self.frame_h = 0
        self.mp_results_buffer = deque(maxlen=snap_frame_diff)

        self.id = ""
        self.timestamps = []
        self.mode = None
        self.trajectory = []
        self.angles = [None, None]
        self.height = None
        self.h_loc = None
        self.v_loc = None

        self.ready_id = False
        self.ready_timestamp = False
        self.ready_mode = False
        self.ready_trajectory = False
        self.ready_angles = False
        self.ready_height = False
        self.ready_hloc = False
        self.ready_vloc = False

    def extract_data(self,
                     frame: np.ndarray,
                     clip_ready: bool,
                     is_recording: bool,
                     is_in_action: bool,
                     full_timestamp: list,
                     idx: int,
                     ball_center: tuple[int, int],
                     results: object
                     ) -> bool:
        """
        Stores all relevant tracking data and sets internal flags
        to indicate which components are ready for serialization.

        :param frame: Current video frame as a NumPy array.
        :type frame: numpy.ndarray
        :param clip_ready: Whether the current video clip is created and ready to be sent to server.
        :type clip_ready: bool
        :param is_recording: Whether the system is currently recording.
        :type is_recording: bool
        :param is_in_action: Whether the action is in desired state ("thrown").
        :type is_in_action: bool
        :param full_timestamp: List of all timestamps of the entire video clip.
        :type full_timestamp: list of str
        :param idx: Index used to select the current mode from `modes` list.
        :type idx: int
        :param ball_center: (x, y) pixel coordinates of the center of the ball in the current frame.
        :type ball_center: tuple of float
        :param results: MediaPipe pose detection results for the current frame.
        :type results: MediaPipe process object
        :return: True if all data components are ready to be serialized; otherwise False.
        :rtype: bool
        """

        self.frame_h, self.frame_w, _ = frame.shape
        self.mp_results_buffer.append(results)

        if is_recording:
            if is_in_action:
                self.get_trajectory(ball_center)
            if not self.ready_mode:
                self.get_mode(idx)
            if not self.ready_angles and self.ready_mode:
                self.get_angles()
            if not self.ready_height:
                self.get_height()
            if not self.ready_hloc and self.ready_mode:
                self.get_horizontal_loc(ball_center)
            if not self.ready_vloc and self.ready_height:
                self.get_vertical_loc(self.height, ball_center)

        if clip_ready:
            self.get_id(id_path=USERID_FILE_PATH)
            self.get_timestamps(full_timestamp)
            if len(self.trajectory) > 0:
                self.ready_trajectory = True
                if self.mode == modes[1]:
                    self.trajectory = None

        if (
                self.ready_id
                and self.ready_timestamp
                and self.ready_mode
                and self.ready_trajectory
                and self.ready_angles
                and self.ready_height
                and self.ready_hloc
                and self.ready_vloc
        ):
            return True
        else:
            return False

    def clear_data(self):
        """
        Resets all extracted data and readiness flags.

        :return: None
        """

        self.id = ""
        self.timestamps = []
        self.mode = None
        self.trajectory = []
        self.angles = [None, None]
        self.height = None
        self.h_loc = None
        self.v_loc = None

        self.ready_id = False
        self.ready_timestamp = False
        self.ready_mode = False
        self.ready_trajectory = False
        self.ready_angles = False
        self.ready_height = False
        self.ready_hloc = False
        self.ready_vloc = False

    def get_id(self, id_path):
        """
        Reads user ID and stores the data.

        :param id_path: Path to the file containing the user ID.
        :type id_path: str
        :return: None
        """

        try:
            with open(id_path, "r", encoding="utf-8") as f:
                self.id = f.read().strip()
        except FileNotFoundError:
            logging.error(f"File not found: {id_path}")
            self.id = "000000"
        except Exception as e:
            logging.error(f"Failed to read file: {e}")
            self.id = "000000"

        self.ready_id = True

    def get_timestamps(self, full_timestamp):
        """
        Extracts key timestamps from the provided full timestamp sequence.

        :param full_timestamp: List of all timestamps of the entire video clip.
        :type full_timestamp: list[str]
        :return: None
        """

        self.timestamps.append(full_timestamp[0])
        self.timestamps.append(full_timestamp[fps * past_t])
        self.timestamps.append(full_timestamp[-1])
        self.ready_timestamp = True

    def get_mode(self, idx):
        """
        Sets the current mode while being recorded.

        :param idx: Index into the global 'modes' list (0 = Side, 1 = Front).
        :type idx: int
        :return: None
        """

        self.mode = modes[idx]
        self.ready_mode = True

    def get_trajectory(self, ball_center):
        """
        Appends the current ball position to the trajectory list.

        :param ball_center: Current (x, y) coordinates of the ball in pixels.
        :type ball_center: tuple[int, int]
        :return: None
        """

        self.trajectory.append(list(ball_center))

    def get_angles(self):
        """
        Calculates elbow and wrist angles from the first frame
        (before release or snapping wrist) in the pose buffer.

        :return: None
        """

        if self.mode == modes[1]:
            self.ready_angles = True
            return

        set_landmarks = self.mp_results_buffer[0].pose_landmarks

        shoulderL = set_landmarks.landmark[12]
        shoulderR = set_landmarks.landmark[11]
        elbowL = set_landmarks.landmark[14]
        elbowR = set_landmarks.landmark[13]
        wristL = set_landmarks.landmark[16]
        wristR = set_landmarks.landmark[15]
        indexL = set_landmarks.landmark[20]
        indexR = set_landmarks.landmark[19]

        shoulderL_coord = (int(shoulderL.x * self.frame_w), int(shoulderL.y * self.frame_h))
        shoulderR_coord = (int(shoulderR.x * self.frame_w), int(shoulderR.y * self.frame_h))
        elbowL_coord = (int(elbowL.x * self.frame_w), int(elbowL.y * self.frame_h))
        elbowR_coord = (int(elbowR.x * self.frame_w), int(elbowR.y * self.frame_h))
        wristL_coord = (int(wristL.x * self.frame_w), int(wristL.y * self.frame_h))
        wristR_coord = (int(wristR.x * self.frame_w), int(wristR.y * self.frame_h))
        indexL_coord = (int(indexL.x * self.frame_w), int(indexL.y * self.frame_h))
        indexR_coord = (int(indexR.x * self.frame_w), int(indexR.y * self.frame_h))

        elbowL_angle = calc_angle(shoulderL_coord, elbowL_coord, wristL_coord)
        elbowR_angle = calc_angle(shoulderR_coord, elbowR_coord, wristR_coord)
        wristL_angle = calc_angle(elbowL_coord, wristL_coord, indexL_coord)
        wristR_angle = calc_angle(elbowR_coord, wristR_coord, indexR_coord)

        elbow, wrist =\
            (elbowL_angle, wristL_angle) if wristL_angle > wristR_angle else (elbowR_angle, wristR_angle)

        self.angles = [elbow, wrist]
        self.ready_angles = True

    def get_height(self):
        """
        Estimates the player's height in pixels from the last frame (release moment) in the pose buffer.
        Height is calculated from nose to heel positions, considering the top of the head.

        :return: None
        """

        thrown_landmarks = self.mp_results_buffer[-1].pose_landmarks

        heelL = thrown_landmarks.landmark[30]
        heelR = thrown_landmarks.landmark[29]
        lipL = thrown_landmarks.landmark[10]
        lipR = thrown_landmarks.landmark[9]
        nose = thrown_landmarks.landmark[0]

        heelL_coord = (int(heelL.x * self.frame_w), int(heelL.y * self.frame_h))
        heelR_coord = (int(heelR.x * self.frame_w), int(heelR.y * self.frame_h))
        lipL_coord = (int(lipL.x * self.frame_w), int(lipL.y * self.frame_h))
        lipR_coord = (int(lipR.x * self.frame_w), int(lipR.y * self.frame_h))
        lipM_coord = calc_midpoint(lipL_coord, lipR_coord)
        nose_coord = (int(nose.x * self.frame_w), int(nose.y * self.frame_h))

        dist = nose_coord[1] - lipM_coord[1]
        top_y = nose_coord[1] + dist * 2

        heightL = abs(top_y - heelL_coord[1])
        heightR = abs(top_y - heelR_coord[1])

        self.height = max(heightL, heightR)
        self.ready_height = True

    def get_horizontal_loc(self, ball_center):
        """
        Computes the horizontal location of the ball relative to the shoulder width.
        Returns normalized value between -1 (left) and 1 (right).

        :param ball_center: Current (x, y) coordinates of the ball in pixels.
        :type ball_center: tuple[int, int]
        :return: None
        """

        if self.mode == modes[0]:
            self.ready_hloc = True
            return

        thrown_landmarks = self.mp_results_buffer[-1].pose_landmarks

        shoulderL = thrown_landmarks.landmark[12]
        shoulderR = thrown_landmarks.landmark[11]

        shoulderL_coord = (int(shoulderL.x * self.frame_w), int(shoulderL.y * self.frame_h))
        shoulderR_coord = (int(shoulderR.x * self.frame_w), int(shoulderR.y * self.frame_h))
        shoulderM_coord = calc_midpoint(shoulderL_coord, shoulderR_coord)

        try:
            h_loc =  (ball_center[0] - shoulderM_coord[0]) / ((shoulderR_coord[0] - shoulderL_coord[0]) / 2)
        except ZeroDivisionError:
            h_loc = 0.0

        self.h_loc = h_loc
        self.ready_hloc = True

    def get_vertical_loc(self, height, ball_center):
        """
        Computes the vertical location of the ball normalized by the player's height.

        :param height: Player's height in pixels.
        :type height: int
        :param ball_center: Current (x, y) coordinates of the ball in pixels.
        :type ball_center: tuple[int, int]
        :return: None
        """

        thrown_landmarks = self.mp_results_buffer[-1].pose_landmarks

        heelL = thrown_landmarks.landmark[30]
        heelR = thrown_landmarks.landmark[29]

        heelL_coord = (int(heelL.x * self.frame_w), int(heelL.y * self.frame_h))
        heelR_coord = (int(heelR.x * self.frame_w), int(heelR.y * self.frame_h))

        distL = abs(heelL_coord[1] - ball_center[1])
        distR = abs(heelR_coord[1] - ball_center[1])
        dist = max(distL, distR)

        try:
            v_loc = dist / height
        except ZeroDivisionError:
            v_loc = 0.0

        self.v_loc = v_loc
        self.ready_vloc = True
