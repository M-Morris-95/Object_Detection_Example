import state_representation as sr
from modulo_components.lifecycle_component import LifecycleComponent
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge
import cv2

class VideoPlayer(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        self.video_path = ''
        self.is_rgb = True

        self.add_parameter(sr.Parameter('video_path', sr.ParameterType.STRING), "Path to the video file.")     
        self.add_parameter(sr.Parameter('is_rgb', sr.ParameterType.BOOL), "Video is RGB?")

        self.video_feed = None
        self.add_output("video_feed", "video_feed", SensorImage)

        self._cv_bridge = CvBridge()
        self._cap = None 

    def on_activate_callback(self) -> bool:
        return True

    def on_activate_callback(self) -> bool:
        video_path = self.get_parameter_value("video_path")

        self._cap = cv2.VideoCapture(video_path)

        if not self._cap.isOpened():
            self.get_logger().error(f"Failed to open video: {video_path}")
            return False

        self.get_logger().info(f"Successfully opened video: {video_path}")
        return True

    def on_deactivate_callback(self) -> bool:
        if self._cap:
            self._cap.release()
        return True

    def on_step_callback(self):
        if self._cap is None:
            self.get_logger().warn("VideoCapture not initialized")
            return

        ret, frame = self._cap.read()

        if not ret:
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # rewind
            ret, frame = self._cap.read()
            if not ret:
                self.get_logger().error("Failed to read video frame after rewind")
                return
            
        if self.get_parameter_value("is_rgb"):
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        self.video_feed = self._cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")


