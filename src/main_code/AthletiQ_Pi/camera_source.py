class CameraSource:
    """
    Handles video capture from different platforms (PC or Raspberry Pi).

    Attributes
    ----------
    platform : str
        Platform type, "pc" for standard webcams, "pi" for Raspberry Pi camera, or "file" for reading video files.
    src : int or str
        Video source index (for PC), or file path (for reading video files).
    rate : int
        Target frame rate in frames per second.
    size : tuple[int, int]
        Desired resolution of the captured frames (width, height).
    cam : object
        Camera object, either `VideoStream` (PC) or `Picamera2` (Pi).
    """

    def __init__(self, platform="pc", src=0, rate=10, size=(1280, 720)):
        self.platform = platform
        self.src = src
        self.rate = rate
        self.size = size
        self.cam = None

    def initialize(self):
        if self.platform == "pc":
            from imutils.video import VideoStream
            self.cam = VideoStream(src=self.src, framerate=self.rate, resolution=self.size)

        elif self.platform == "pi":
            from picamera2 import Picamera2
            self.cam = Picamera2()
            config = self.cam.create_video_configuration(
                main={
                    "size": self.size,
                    "format": "RGB888"
                },
                controls={
                    "FrameDurationLimits": (
                        int(1_000_000 / self.rate),
                        int(1_000_000 / self.rate)
                    )
                },
                buffer_count=2
            )
            self.cam.configure(config)

        elif self.platform == "file":
            import cv2
            self.cam = cv2.VideoCapture(self.src)
            if not self.cam.isOpened():
                raise IOError(f"Cannot open video file: {self.src}")

        else:
            raise ValueError(f"Unknown platform: {self.platform}")

    def start(self):
        if self.platform in ("pc", "pi"):
            self.cam.start()

    def read(self):
        if self.platform == "pc":
            return self.cam.read()
        elif self.platform == "pi":
            return self.cam.capture_array("main")
        elif self.platform == "file":
            ret, frame = self.cam.read()
            if not ret:
                return None
            return frame
        else:
            return None

    def stop(self):
        if self.platform == "pc":
            self.cam.stop()
        elif self.platform == "pi":
            self.cam.stop()
        elif self.platform == "file":
            self.cam.release()
