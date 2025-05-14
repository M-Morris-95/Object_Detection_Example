import state_representation as sr
from modulo_components.lifecycle_component import LifecycleComponent
from std_msgs.msg import String
from modulo_core import EncodedState
import clproto
import json
import numpy as np

from rclpy.duration import Duration
from copy import deepcopy

class YoloToMarker(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        # inputs
        self.json_input = ""
        self.add_input("json_input", "json_input", String)

        # outputs
        self._marker_pose = sr.CartesianPose("object", "camera_frame") #pose with name object, referenced to camera frame
        self.add_output("marker_pose", "_marker_pose", EncodedState)

        # parameters
        self.to_find = 'cup'
        self.fov = [69, 42]
        self.image_size = [840, 480]
        self.object_distance = 0.6
        
        self.add_parameter(sr.Parameter("to_find", sr.ParameterType.STRING), "thing to find and track")
        self.add_parameter(sr.Parameter("fov", [69.0, 42.0], sr.ParameterType.DOUBLE_ARRAY), "Camera FoV in X")
        self.add_parameter(sr.Parameter("image_size", [840, 480], sr.ParameterType.DOUBLE_ARRAY), "Size of image (pixels)")
        self.add_parameter(sr.Parameter("object_distance", 0.6, sr.ParameterType.DOUBLE), "Target position (Z)")
        
    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        # validate an incoming parameter value according to some criteria
        return True
    
    def on_configure_callback(self) -> bool:
        # configuration steps before running
        self.fov = self.get_parameter_value("fov")
        self.fov = np.radians(np.asarray(self.fov))

        self.object_distance = self.get_parameter_value("object_distance")
        return True

    def on_activate_callback(self) -> bool:
        # activation steps before running
        return True

    def on_deactivate_callback(self) -> bool:
        # deactivatio steps
        return True

    def __centre_pt_to_position(self, xy):
        # function to find find the 3d position of an object based on position in the image.
        image_size = np.asarray(self.image_size)
        ratio = (xy - image_size/2)/(image_size/2) 
        position = ratio * self.object_distance * np.tan(self.fov/2)    

        position = (np.append(position, self.object_distance)).reshape((3,1))
        return position

    def on_step_callback(self):
        try:

            data = json.loads(self.json_input)
            if not isinstance(data, dict):
                self.get_logger().error("Invalid JSON format: expected a dictionary of bounding boxes.")
                return

            position = {}
            # Compute 3D positions for all detected objects
            for key, coords in data.items():
                if not isinstance(coords, (list, tuple)) or len(coords) != 4:
                    self.get_logger().warn(f"Skipping malformed bounding box for '{key}': {coords}")
                    continue

                centre_of_bbox = np.asarray([
                    (coords[0] + coords[2]) / 2,
                    (coords[1] + coords[3]) / 2
                ])
                position[key] = self.__centre_pt_to_position(centre_of_bbox)

            if not position:
                self.get_logger().warn("No valid objects found in YOLO output.")
                return

            # Output pose for target object
            to_find = self.get_parameter_value("to_find")
            target_key = f"{to_find}_1"
            if target_key in position:
                self._marker_pose.set_position(position[target_key])
            else:
                self.get_logger().warn(f"Target object '{target_key}' not found in detected objects.")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON input: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in on_step_callback: {e}")

