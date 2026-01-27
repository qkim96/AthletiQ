from utils import *


class PoseDetector:
    """
    Detects and tracks human pose landmarks in video frames using MediaPipe,
    and computes key coordinates.

    Attributes
    ----------
    mp_pose : MediaPipe object
        MediaPipe pose module used for landmark detection.
    mp_drawing : MediaPipe object
        MediaPipe drawing utilities for visualization.
    pose : MediaPipe object
        MediaPipe Pose object configured with specified detection and tracking confidence.
    """

    def __init__(self, min_detection_conf=min_det_conf, min_tracking_conf=min_tra_conf):
        """
        Initializes the PoseDetector instance with MediaPipe pose detection and tracking settings.

        :param min_detection_conf: Minimum confidence for detecting pose landmarks.
        :type min_detection_conf: float
        :param min_tracking_conf: Minimum confidence for tracking pose landmarks across frames.
        :type min_tracking_conf: float
        """

        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=min_detection_conf,
            min_tracking_confidence=min_tracking_conf
        )

    def process_frame(self, frame):
        """
        Processes a video frame to detect pose landmarks and extract coordinates.

        :param frame: Input video frame (BGR format).
        :type frame: numpy.ndarray
        :return: Tuple containing MediaPipe results and coordinates of key points or None if detection fails.
        :rtype: tuple[MediaPipe object or None, tuple[tuple[int, int], ...] or None]
        """

        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)
        coords = self.get_coord(frame, results)

        return results, coords

    def get_coord(self, frame, results):
        """
        Extracts pixel coordinates of nose, left hand, right hand, and the center of mass.

        :param frame: Input video frame.
        :type frame: numpy.ndarray
        :param results: MediaPipe pose detection results.
        :type results: MediaPipe object
        :return: Tuple of 4 coordinates or None if landmarks are not detected.
        :rtype: tuple[tuple[int, int], ...] or None
        """

        h, w, _ = frame.shape

        try:
            nose = results.pose_landmarks.landmark[0]
            handL = results.pose_landmarks.landmark[20]
            handR = results.pose_landmarks.landmark[19]

            nose_coord = (int(nose.x * w), int(nose.y * h))
            handL_coord = (int(handL.x * w), int(handL.y * h))
            handR_coord = (int(handR.x * w), int(handR.y * h))

            points = np.array([
                [nose_coord[0], nose_coord[1]],
                [handL_coord[0], handL_coord[1]],
                [handR_coord[0], handR_coord[1]],
            ], dtype=np.int32)

            cmass_coord = self.get_cmass(points, (w, h))

            coords = (nose_coord, handL_coord, handR_coord, cmass_coord)
        except AttributeError:
            coords = None

        return coords

    @staticmethod
    def get_cmass(points, res):
        """
        Computes the center of mass from given landmark points.

        :param points: Array of (x, y) coordinates of selected landmarks.
        :type points: numpy.ndarray
        :param res: Frame resolution (width, height).
        :type res: tuple[int, int]
        :return: Center of mass pixel coordinates.
        :rtype: tuple[int, int]
        """

        w, h = res

        M = cv2.moments(points)

        try:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        except ZeroDivisionError:
            cx, cy = int(w * 0.5), int(h * 0.5)

        return cx, cy

    @staticmethod
    def body_position_LR(frame, coords):
        """
        Determines whether the player's center of mass is close to the left or right margin of the frame.

        :param frame: Video frame for width reference.
        :type frame: numpy.ndarray
        :param coords: Tuple of key coordinates, including the center of mass.
        :type coords: tuple[tuple[int, int], ...] or None
        :return: Tuple of flags, True if center of mass is near left/right boundary.
        :rtype: tuple[bool, bool]
        """

        if coords is None:
            return False, False

        _, w, _ = frame.shape
        _, _, _, (cmass_x, _) = coords

        thresh_l = int(w * width_cutoff)
        thresh_r = int(w * (1 - width_cutoff))

        if cmass_x < thresh_l:
            return True, False
        elif cmass_x > thresh_r:
            return False, True
        else:
            return False, False

    def release(self):
        """
        Releases MediaPipe resources to free memory.
        """

        self.pose.close()
