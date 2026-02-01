from utils import *


class ObjectLocalizer:
    """
    Localizes and tracks the object in real-time video frames using a Kalman filter.

    The object is detected via HSV thresholding and morphological image processing,
    and its center and radius are extracted from contours. The Kalman filter integrates
    previous state estimates with noisy observations to produce a smoothed estimate of
    the object's position, handling temporary occlusions or missed detections.

    Attributes
    ----------
    A_t : numpy.ndarray of shape (2, 2)
        State transition model matrix.
    G : numpy.ndarray of shape (2, 1)
        Process noise gain matrix.
    R_t : numpy.ndarray of shape (2, 2)
        Process noise covariance matrix.
    C_t : numpy.ndarray of shape (1, 2)
        Observation model matrix.
    Q_t : numpy.ndarray of shape (1, 1)
        Observation noise covariance matrix.
    mu_t : numpy.ndarray of shape (2, 2)
        Current state estimate.
    sigma_t : numpy.ndarray of shape (2, 2)
        Current estimate covariance matrix.
    red_lower1 : tuple[int, int, int]
        Lower HSV bound for low-hue red exclusion.
    red_upper1 : tuple[int, int, int]
        Upper HSV bound for low-hue red exclusion.
    red_lower2 : tuple[int, int, int]
        Lower HSV bound for high-hue red exclusion.
    red_upper2 : tuple[int, int, int]
        Lower HSV bound for high-hue red exclusion.
    orange_lower : tuple[int, int, int]
        Lower HSV bound for detecting the object.
    orange_upper : tuple[int, int, int]
        Upper HSV bound for detecting the object.
    enhance_lower : tuple[int, int, int]
        Lower LAB bound for identifying pixels similar to orange for color enhancement prior to object detection.
    enhance_upper : tuple[int, int, int]
        Upper LAB bound for identifying pixels similar to orange for color enhancement prior to object detection.
    found : bool
        True if the object was detected at least once (object initial location found).
    radius : int or None
        Radius of the detected contour.
    center : tuple[int, int] or tuple[None, None]
        Observed center pixel coordinates of the object in the current frame.
    x_bel : int or None
        Estimated posterior of the object x-coordinate after applying Kalman filter.
    y_bel : int or None
        Estimated posterior of the object y-coordinate after applying Kalman filter.
    z : numpy.ndarray of shape (1, 2) or None
        Accepted measurement by ignoring noisy observation.
    prev_z : numpy.ndarray of shape (2,) or None
        Most recently accepted measurement.
    jump_cnt : int
        Counter for tracking the number of consecutive measurements exceeding the displacement threshold.
    """

    def __init__(self):
        """
        Initializes the ObjectLocalizer instance with Kalman filter parameters,
        HSV thresholds, and default tracking states.
        """

        self.A_t = np.array([[1, 1], [0, 1]])
        self.G = np.array([[0.5], [1]])
        self.R_t = self.G.dot(self.G.transpose())
        self.C_t = np.array([[1, 0]])
        self.Q_t = np.array([[1]])
        self.mu_t = np.array([[0, 0], [0, 0]])
        self.sigma_t = np.array([[0, 0], [0, 0]])

        self.red_lower1 = (0, 100, 50)
        self.red_upper1 = (10, 255, 255)
        self.red_lower2 = (170, 100, 50)
        self.red_upper2 = (180, 255, 255)
        self.orange_lower = (10, 90, 60)
        self.orange_upper = (28, 255, 255)
        self.enhance_lower = (0, 135, 130)
        self.enhance_upper = (255, 185, 200)

        self.found = False
        self.radius = None
        self.center = (None, None)
        self.x_bel, self.y_bel = None, None
        self.z = None
        self.prev_z = None
        self.jump_cnt = 0

    def kalman_filter(self, mu_prev, sigma_prev, z):
        """
        Performs one iteration of the Kalman filter, given the previous state and current observation.

        :param mu_prev: Previous state estimate.
        :type mu_prev: np.ndarray of shape (2, 2)
        :param sigma_prev: Previous estimate covariance estimate.
        :type sigma_prev: np.ndarray of shape (2, 2)
        :param z: Current measurement (observation).
        :type z: np.ndarray of shape (1, 2) or None
        :return: Posterior state estimate and covariance.
        :rtype: tuple[np.ndarray, np.ndarray]
        """

        mu_bar = self.A_t.dot(mu_prev)
        sigma_bar = self.A_t.dot(sigma_prev).dot(self.A_t.transpose()) + self.R_t
        if z is None:
            return mu_bar, sigma_bar
        else:
            K_t = sigma_bar.dot(
                self.C_t.transpose()).dot(
                inv(self.C_t.dot(sigma_bar).dot(self.C_t.transpose()) + self.Q_t))
            mu = mu_bar + K_t.dot(z - self.C_t.dot(mu_bar))
            sigma = (np.identity(2) - K_t.dot(self.C_t)).dot(sigma_bar)
            return mu, sigma

    def process_frame(self, frame):
        """
        Processes a single video frame to detect the object by applying HSV threshold,
        LAB enhancement, and morphological operations on the binary mask, and to track
        object by using a Kalman filter on the prior state and the current observation.

        :param frame: Current video frame (BGR format).
        :type frame: np.ndarray
        :return: Prior center, posterior center, and the detected object radius.
        :rtype: tuple[tuple[int, int], tuple[int, int], int] or tuple[tuple[None, None], tuple[None, None], None]
        """

        # Preprocessing
        blr = cv2.GaussianBlur(frame, (11, 11), 0)  # apply Gaussian blur to reduce noise
        hsv = cv2.cvtColor(blr, cv2.COLOR_BGR2HSV)  # convert frame from BGR to HSV space
        lab = cv2.cvtColor(blr, cv2.COLOR_BGR2LAB)  # convert frame from BGR to LAB space
        L, A, B = cv2.split(lab)  # split LAB channels

        # Mask for red-exclusion and bias for orange-enhancement
        red_mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        orange_bias = (B.astype(np.int16) - A.astype(np.int16)) > -15

        # Enhancement mask using LAB (robust to shadows, highlights, changes in lighting)
        lab_mask = cv2.inRange(lab, self.enhance_lower, self.enhance_upper)
        lab_mask = cv2.bitwise_and(lab_mask, cv2.bitwise_not(red_mask))
        lab_mask = cv2.bitwise_and(lab_mask, orange_bias.astype(np.uint8) * 255)

        # Exclude red regions from the enhancement mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (17, 17))  # used moderately big kernel for exclusion
        red_center = (((hsv[..., 0] < 8) | (hsv[..., 0] > 170)) & (hsv[..., 1] > 90)).astype(np.uint8) * 255
        red_expand = cv2.dilate(red_center, kernel)  # from the region of high red-pixel density, expand the area
        lab_mask[red_expand > 0] = 0  # exclude expanded red region from the mask

        # Enhance HSV in pixels near orange color
        hsv = hsv.astype(np.float32)
        hsv[..., 1][lab_mask > 0] *= 2.8  # restore saturation
        hsv[..., 2][lab_mask > 0] *= 1.6  # restore brightness
        hsv[..., 0][lab_mask > 0] = 15    # normalize hue toward orange
        hsv = np.clip(hsv, 0, 255).astype(np.uint8)

        # Create detection mask
        mask = cv2.inRange(hsv, self.orange_lower, self.orange_upper)  # final HSV threshold mask
        mask[(hsv[..., 1] < 60) & (lab_mask == 0)] = 0  # exclude pale pixels only outside enhance mask
        mask = cv2.erode(mask, None, iterations=2)   # apply erosion to remove small noises in mask
        mask = cv2.dilate(mask, None, iterations=2)  # apply dilation to fill in gaps in mask

        # Extract contours from mask for object detection
        cntr = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # find contours in mask
        cntr = imutils.grab_contours(cntr)  # grab contours as a list for further processing

        # If measurement exists, select the right object
        if len(cntr) > 0:
            self.found = True  # initial detection (t=0 frame in Kalman filter), never set to False from this point
            c = max(cntr, key=cv2.contourArea)

            ((x, y), self.radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"] > 0:
                measurement = np.array([x, y])

                # Ignore sudden "jumps" of the object due to noisy measurement
                # and accept it if the confidence sufficiently builds up
                if self.prev_z is None:
                    self.z = measurement.reshape(1, 2)
                    self.prev_z = measurement
                    self.jump_cnt = 0
                    self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                else:
                    jump_dist = np.linalg.norm(measurement - self.prev_z)

                    if jump_dist < max_jump:
                        self.z = measurement.reshape(1, 2)
                        self.prev_z = measurement
                        self.jump_cnt = 0
                        self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    else:
                        self.jump_cnt += 1

                        if self.jump_cnt > max_ignore:
                            self.z = measurement.reshape(1, 2)
                            self.prev_z = measurement
                            self.jump_cnt = 0
                            self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                        else:
                            self.z = None
                            self.center = (None, None)

        # Update posterior with prior and current observation
        if self.found and (len(cntr) > 0):
            self.mu_t, self.sigma_t = self.kalman_filter(self.mu_t, self.sigma_t, self.z)
            self.x_bel, self.y_bel = int(self.mu_t[0][0]), int(self.mu_t[0][1])

        # Update posterior without observation
        elif self.found and (len(cntr) <= 0):
            self.mu_t, self.sigma_t = self.kalman_filter(self.mu_t, self.sigma_t, None)
            self.x_bel, self.y_bel = int(self.mu_t[0][0]), int(self.mu_t[0][1])
            self.center = (None, None)

        return self.center, (self.x_bel, self.y_bel), self.radius
