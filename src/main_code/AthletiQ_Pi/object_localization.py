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
    orangeLower : tuple[int, int, int]
        Lower HSV bound for detecting the object.
    orangeUpper : tuple[int, int, int]
        Upper HSV bound for detecting the object.
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

        self.orangeLower = (10, 150, 100)
        self.orangeUpper = (40, 255, 255)

        self.found = False
        self.radius = None
        self.center = (None, None)
        self.x_bel, self.y_bel = None, None

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
        Processes a single video frame to detect the object by applying HSV threshold
        and morphological operations on the binary mask, and to track object by using
        the Kalman filter on the prior state and the current observation.

        :param frame: Current video frame (BGR format).
        :type frame: np.ndarray
        :return: Prior center, posterior center, and the detected object radius.
        :rtype: tuple[tuple[int, int], tuple[int, int], int] or tuple[tuple[None, None], tuple[None, None], None]
        """

        blr = cv2.GaussianBlur(frame, (11, 11), 0)  # apply Gaussian blur to reduce noise
        hsv = cv2.cvtColor(blr, cv2.COLOR_BGR2HSV)  # convert frame from BGR to HSV space
        mask = cv2.inRange(hsv, self.orangeLower, self.orangeUpper)  # create a mask to isolate orange color
        mask = cv2.erode(mask, None, iterations=2)   # apply erosion to remove small noises in mask
        mask = cv2.dilate(mask, None, iterations=2)  # apply dilation to fill in gaps in mask
        cntr = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # find contours in mask
        cntr = imutils.grab_contours(cntr)  # grab contours as a list

        # Found object for the first time (t=0 frame in Kalman filter)
        if len(cntr) > 0:
            self.found = True  # never set to False from this point
            c = max(cntr, key=cv2.contourArea)
            ((x, y), self.radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # Update posterior with prior and current observation
        if self.found and (len(cntr) > 0):
            self.mu_t, self.sigma_t = self.kalman_filter(self.mu_t, self.sigma_t, np.array([[x, y]]))
            self.x_bel, self.y_bel = int(self.mu_t[0][0]), int(self.mu_t[0][1])

        # Update posterior without observation
        elif self.found and (len(cntr) <= 0):
            self.mu_t, self.sigma_t = self.kalman_filter(self.mu_t, self.sigma_t, None)
            self.x_bel, self.y_bel = int(self.mu_t[0][0]), int(self.mu_t[0][1])

        return self.center, (self.x_bel, self.y_bel), self.radius
