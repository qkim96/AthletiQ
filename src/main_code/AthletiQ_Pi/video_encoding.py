from utils import *


class VideoEncoder:
    """
    Clip the desired event by detecting action start/end, and encode the video frame buffer into H.264 format.

    Attributes
    ----------
    past_duration : int
        Duration (in seconds) to keep past frames before an action trigger.
    future_duration : int
        Duration (in seconds) to store frames after an action trigger.
    final_duration : int
        Duration (in seconds) to capture frames before ending the clip, once the state transitions from "Thrown" state.
    past_buffer : deque
        Buffer storing past frames with timestamps.
    future_frames : list of numpy.ndarray
        List storing future frames after an action trigger.
    future_max_len : int
        Maximum number of frames to store in `future_frames`.
    is_recording : bool
        True while capturing frames for a clip.
    is_in_action : bool
        True during the "Thrown" action state.
    shoot_time : float
        Time (in seconds) when action started.
    shoot_end_time : float
        Time (in seconds) when action ended.
    finalized_time: int
        Last time a clip was finalized.
    """

    def __init__(self, past_duration=past_t, future_duration=future_t, final_duration=final_t):
        """
        Initializes the VideoEncoder instance by constructing buffers with specified duration
        and state flags used for timing clip creations.

        :param past_duration: Duration (in seconds) of frames to retain before the action trigger.
        :type past_duration: int
        :param future_duration: Maximum duration (in seconds) of frames to capture after the action trigger.
        :type future_duration: int
        :param final_duration: Duration (in seconds) to continue capturing frames after the action ends.
        :type final_duration: int
        """

        self.past_duration = past_duration
        self.future_duration = future_duration
        self.final_duration = final_duration

        self.past_buffer = deque(maxlen=int(fps * past_duration))
        self.future_frames = []
        self.future_max_len = int(fps * future_duration)
        
        self.is_recording = False
        self.is_in_action = False
        self.shoot_time = None
        self.shoot_end_time = None
        self.finalized_time = time.time()

    def update_buffer(self, frame, s_curr, s_prev):
        """
        Updates the past/future buffers with the current frame
        and handles the timing for encoding them into H.264.

        :param frame: Current video frame as a NumPy array.
        :type frame: numpy.ndarray
        :param s_curr: Current action state index used for detecting triggers.
        :type s_curr: int
        :param s_prev: Previous action state index used for detecting triggers.
        :type s_prev: int
        :return: Tuple containing H.264-encoded bytes and timestamps if a clip is finalized; otherwise (None, None).
        :rtype: tuple[bytes or None, list[str] or None]
        """

        now = datetime.now()
        timestamp = now.strftime("%Y-%m-%d %H:%M:%S")

        # If not recording, check action start trigger
        # to start recording or queue current frame into past buffer
        if not self.is_recording:
            if self.is_triggered_start(s_curr, s_prev):
                self.is_recording = True
                self.is_in_action = True
            else:
                self.past_buffer.append((timestamp, frame))

        # While recording, append current frames to list of frames to record
        # and check for end trigger
        if self.is_recording:
            self.future_frames.append((timestamp, frame))
            self.check_shoot_end(s_curr, s_prev)

            if self.is_triggered_end():
                output_bytes, timestamps = self.finalize()
                self.is_recording = False
                self.is_in_action = False
                self.shoot_time = None
                self.shoot_end_time = None

                if output_bytes:
                    return output_bytes, timestamps

        return None, None

    def is_triggered_start(self, s_curr, s_prev):
        """
        Detects the start of an action based on the action state transition.

        :param s_curr: Current action state index.
        :type s_curr: int
        :param s_prev: Previous action state index.
        :type s_prev: int
        :return: True if action start is detected after 'past_duration'; otherwise False.
        :rtype: bool
        """

        if time.time() - self.finalized_time < self.past_duration:
            return False

        self.shoot_time = time.time()

        return s_prev == 1 and s_curr == 2

    def is_triggered_end(self):
        """
        Determines whether the action has ended or maximum future frames are captured.

        :return: True if recording should stop; otherwise False.
        :rtype: bool
        """

        is_full = len(self.future_frames) >= self.future_max_len

        if self.is_in_action:    # still in "thrown" state
            shoot_done = False
            if is_full:          # but video length hits the max time
                self.shoot_end_time = time.time()
        else:                    # "thrown" state has ended
            shoot_done = time.time() - self.shoot_end_time > self.final_duration

        # Either video length hit the max
        # or a set amount time has passed since the action of interest ("thrown") has ended
        return is_full or shoot_done

    def check_shoot_end(self, s_curr, s_prev):
        """
        Updates `shoot_end_time` if the action has ended.

        :param s_curr: Current action state index.
        :type s_curr: int
        :param s_prev: Previous action state index.
        :type s_prev: int
        :return: None
        """

        if self.is_in_action and s_prev == 2 and s_curr != 2:
            self.is_in_action = False
            self.shoot_end_time = time.time()

    def clear_buffers(self):
        """
        Clears both past and future frame buffers.

        :return: None
        """

        self.future_frames = []
        self.past_buffer.clear()

    def finalize(self):
        """
        Finalizes the video clip by combining past and future frames, encoding to H.264, and resetting buffers.
        Discards clips if recorded before 'past_duration' or if shorter than `duration_thresh`.

        :return: Tuple containing H.264-encoded bytes and corresponding timestamps, or (None, None) if discarded.
        :rtype: tuple[bytes or None, list[str] or None]
        """

        frames = [f for _, f in self.past_buffer] + [f for _, f in self.future_frames]
        timestamps = [t for t, _ in self.past_buffer] + [t for t, _ in self.future_frames]

        if len(self.past_buffer) < self.past_buffer.maxlen:
            print("Video clip discarded (record started too early)")
            self.clear_buffers()
            return None, None

        self.clear_buffers()

        if self.shoot_end_time - self.shoot_time <= duration_thresh:
            print("Video clip discarded (clip too short)")
            return None, None

        self.finalized_time = time.time()
        print("Video clip created")
        return self.encode_h264(frames), timestamps

    @staticmethod
    def encode_h264(frames) -> bytes:
        """
        Encodes a list of frames as H.264 bytes.

        :param frames: List of frames as NumPy arrays.
        :type frames: list[numpy.ndarray]
        :return: Bytes of the encoded H.264 video.
        :rtype: bytes
        """

        h, w, _ = frames[0].shape

        # Create in-memory buffer to store encoded H.264 video
        h264_buffer = io.BytesIO()

        # Create a container for H.264 to write H.264 data into the memory buffer
        container = av.open(h264_buffer, mode="w", format="h264")

        # Create a video stream to the container
        stream = container.add_stream("h264", rate=fps)
        stream.width = w
        stream.height = h
        stream.pix_fmt = "yuv420p"

        # Encode each frame
        for f in frames:
            # Convert NumPy frame (BGR) to PyAV frame (RGB)
            frame = av.VideoFrame.from_ndarray(f, format="bgr24")

            # Encode the frame and mux generated packets into the container
            for packet in stream.encode(frame):
                container.mux(packet)

        # Flush encoder to ensure any remaining delayed frames are written
        for packet in stream.encode():
            container.mux(packet)

        container.close()

        return h264_buffer.getvalue()
