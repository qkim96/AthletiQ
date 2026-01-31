from utils import *


class StateEstimator:
    """
    Uses a Hidden Markov Model (HMM) with factorized geometric features derived
    from ball position and pose landmarks to estimates the current action state
    from visual observations.

    HMM forward filtering and 1-step Viterbi-style local decoding are combined
    with Maximum a Posteriori (MAP) deterministic decision rule to produce a
    stable estimate of the hidden state over time.

    Attributes
    ----------
    s_t : int or None
        Estimated current state index.
    o_t : str or None
        Current observation encoded as a binary feature string.
    s_prev : int or None
        Estimated previous state index.
    pi : numpy.ndarray
        Initial state distribution.
    A : numpy.ndarray
        State transition probability matrix.
    factorized_B : numpy.ndarray
        Factorized emission probability matrix.
    B : numpy.ndarray
        Emission probability matrix.
    alpha_t : numpy.ndarray or None
        Forward variable vector.
    delta_t : numpy.ndarray or None
        Viterbi log-probability vector.
    alpha_conf : float or None
        Confidence score for forward filtering.
    """

    def __init__(self):
        """
        Initializes the StateEstimator instance and all HMM parameters.

        Sets up state, observation, transition probabilities and emission
        probabilities derived from factorized features, and initializes
        forward and Viterbi variables.
        """

        self.s_t = None
        self.o_t = None
        self.s_prev = None

        self.pi = np.array([0.85, 0.1, 0.05])

        self.A = np.array([
            [0.90, 0.10, 0.00],
            [0.05, 0.20, 0.75],
            [0.30, 0.05, 0.65]
        ])

        self.factorized_B = np.array([
            [0.20, 0.05, 0.60],
            [0.90, 0.90, 0.90],
            [0.80, 0.45, 0.05]
        ])

        self.B = self.init_emission(self.factorized_B)

        self.alpha_t = None
        self.delta_t = None
        self.alpha_conf = None

    @staticmethod
    def init_emission(factorized_B):
        """
        Builds the full emission probability matrix from factorized features.

        :param factorized_B: Probability of each feature being true per state.
        :type factorized_B: numpy.ndarray
        :return: Emission probability matrix.
        :rtype: numpy.ndarray
        """

        n_state = len(states)
        n_feature = factorized_B.shape[1]
        n_col = 2 ** n_feature

        B = np.zeros((n_state, n_col))

        for obs in range(n_col):  # for all possible observations (= 2^{number_of_factorized_features})
            bits = [int(bit) for bit in f"{obs:0{n_feature}b}"]  # all 3-bit T/F combinations

            for state in range(n_state):  # for each hidden state
                emis_prob = 1.0
                for feat in range(n_feature):  # for each factorized feature
                    p_true = factorized_B[state, feat]  # probability of a feature being true
                    emis_prob *= (p_true ** bits[feat]) * ((1 - p_true) ** (1 - bits[feat]))  # Bernoulli distribution

                B[state, obs] = emis_prob

        return B

    def observe(self, ball_pos, radius, coords):
        """
        Extracts binary geometric features from the ball and pose landmarks
        and encodes them as a 3-bit observation string for the HMM.

        :param ball_pos: (x, y) pixel coordinates of the ball.
        :type ball_pos: tuple[int, int]
        :param radius: Radius of the detected ball.
        :type radius: int
        :param coords: Pose landmarks pixel coordinates.
        :type coords: tuple[tuple[int, int], ...]
        :return: None
        """

        if not radius or not coords:
            self.o_t = None
            return

        (ball_x, ball_y) = ball_pos
        (nose_x, nose_y), *hands, _ = coords

        ball = np.array(ball_pos)
        dist_thresh = radius * 3

        # Factorized features as booleans
        feat0 = ball_y < nose_y
        feat1 = any(hand_y < nose_y for _, hand_y in hands)
        feat2 = any(np.linalg.norm(ball - np.array(hand)) < dist_thresh for hand in hands)

        # A set of features (booleans) encoded into an observation (binary string)
        self.o_t = f"{int(feat0)}{int(feat1)}{int(feat2)}"

    def update_forward(self):
        """
        Performs forward filtering and updates the forward variable
        probability vector for the current observation.

        :return: None
        """

        o_t_idx = int(self.o_t, 2)
        B_o_t = self.B[:, o_t_idx]

        # At t=0
        if self.alpha_t is None:
            self.alpha_t = B_o_t * self.pi
            self.s_prev = np.argmax(self.alpha_t)

        # At t>0
        else:
            alpha_prev = self.alpha_t.copy()
            self.alpha_t = B_o_t * (alpha_prev.dot(self.A))

        # Normalize to avoid underflow & compute confidence
        self.alpha_t /= np.sum(self.alpha_t)
        alpha_weighted = self.alpha_t * self.A[self.s_prev, :]
        self.alpha_conf = alpha_weighted[np.argmax(self.alpha_t)] / np.sum(alpha_weighted)

    def update_viterbi(self):
        """
        Performs Viterbi-style decoding and updates the Viterbi
        log-probability vector for the current observation.

        :return: None
        """

        o_t_idx = int(self.o_t, 2)
        B_o_t = self.B[:, o_t_idx]
        eps = 1e-12

        # At t=0
        if self.delta_t is None:
            self.delta_t = np.log(B_o_t + eps) + np.log(self.pi + eps)
            self.s_prev = np.argmax(self.delta_t)

        # At t>0
        else:
            # Normalize with log space to avoid underflow
            delta_prev = self.delta_t.copy()
            self.delta_t = (np.log(B_o_t + eps)
                            + np.max(delta_prev[:, None] + np.log(self.A + eps), axis=0)
                            + np.log(self.A[self.s_prev, :] + eps))

    def estimate(self):
        """
        Estimates the current hidden state using both Forward and Viterbi algorithms.

        Updates the hidden state at current time step `s_t` by choosing between the
        forward estimate and Viterbi estimate depending on the forward filtering
        confidence score.

        :return: None
        """

        if self.o_t is None:
            self.s_t = -1
            return

        self.s_prev = self.s_t

        # Update alpha/delta
        self.update_forward()
        self.update_viterbi()

        # State estimation using Forward/Viterbi algorithm
        s_bel_forward = np.argmax(self.alpha_t)
        s_bel_viterbi = np.argmax(self.delta_t)

        # State selection based on forward confidence
        if self.alpha_conf > 0.1:
            self.s_t = s_bel_forward
        else:
            self.s_t = s_bel_viterbi
