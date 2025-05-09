import state_representation as sr
from modulo_components.lifecycle_component import LifecycleComponent
from std_msgs.msg import String
from modulo_core import EncodedState
import clproto
import json
import numpy as np
from copy import deepcopy

class YoloToMarker(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        self.json_input = ""
        self.add_input("json_input", "json_input", String)

        self.to_find = 'cup'
        self.add_parameter(sr.Parameter("to_find", sr.ParameterType.STRING), "thing to find and track")

        self._state_command = sr.CartesianState()
        self.add_output("state_command", "_state_command", EncodedState, clproto.MessageType.CARTESIAN_STATE_MESSAGE)

        self.fov_x = 69
        self.fov_y = 42

        self.start_pos_x = 0
        self.start_pos_y = 0.6
        self.start_pos_z = 0
        self.camera_height = 0.6

        self.img_shape = np.asarray([840, 480])
        self.add_parameter(sr.Parameter("to_find", sr.ParameterType.STRING), "thing to find and track")
        self.add_parameter(sr.Parameter("fov_x", 69.0, sr.ParameterType.DOUBLE), "Camera FoV in X")
        self.add_parameter(sr.Parameter("fov_y", 42.0, sr.ParameterType.DOUBLE), "Camera FoV in Y")
        self.add_parameter(sr.Parameter("start_pos_x", 0.0, sr.ParameterType.DOUBLE), "Centre position of camera (x)")
        self.add_parameter(sr.Parameter("start_pos_y", 0.6, sr.ParameterType.DOUBLE), "Centre position of camera (y)")
        self.add_parameter(sr.Parameter("camera_height", 1.0, sr.ParameterType.DOUBLE), "Centre position of camera (z)")
        self.add_parameter(sr.Parameter("goal_z", 0.0, sr.ParameterType.DOUBLE), "Target position (Z)")

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        # validate an incoming parameter value according to some criteria
        return True

    def on_configure_callback(self) -> bool:
        self.fov_x = self.get_parameter_value("fov_x")
        self.fov_y = self.get_parameter_value("fov_y")

        self.start_pos_x = self.get_parameter_value("start_pos_x")
        self.start_pos_y = self.get_parameter_value("start_pos_y")
        self.camera_height = self.get_parameter_value("camera_height")
        self.goal_z = self.get_parameter_value("goal_z")

        self.camera_height

        fov = np.radians(np.asarray([self.fov_x, self.fov_y]))
        
        self.camera_pos = np.asarray([self.start_pos_x, self.start_pos_y, self.camera_height]) # m
        

        self.get_logger().info(
            f'fov:{tuple(f"{v:.2f}" for v in fov)},\t'
            f'camera_height:{self.camera_height},\t'
        )

        
        self.pix_xy = self.camera_height * np.tan(fov/2)
    
        # configuration steps before activation
        return True

    def on_activate_callback(self) -> bool:
        # activation steps before running
        return True

    def on_deactivate_callback(self) -> bool:
        return True

    def __centre_pt_to_position(self,xy):
        
        ratio = (xy - self.img_shape/2)/(self.img_shape/2) # ratio
        ratio[1] = -ratio[1]

        return ratio * self.pix_xy

    def on_step_callback(self):
        try:
            data = json.loads(self.json_input)

            results = {
                key: {
                    'centre': np.asarray([
                        (coords[0] + coords[2]) / 2,
                        (coords[1] + coords[3]) / 2
                    ]),
                    'position': self.__centre_pt_to_position(
                        np.asarray([
                            (coords[0] + coords[2]) / 2,
                            (coords[1] + coords[3]) / 2
                        ])
                    )
                }
                for key, coords in data.items()
            }
            
            to_find = self.get_parameter_value("to_find")

            position = results[to_find + '_1']['position']
            position = (np.append(position, self.goal_z)).reshape((3,1))
            position[:2, 0] += self.camera_pos[:2]
                       
            orientation = np.zeros((4, 1))
            linear_velocity = np.zeros((3, 1))
            angular_velocity = np.zeros((3, 1))
            
            command = deepcopy(self._state_command)
            command.set_position(position)
            command.set_orientation(orientation)
            command.set_linear_velocity(linear_velocity)
            command.set_angular_velocity(angular_velocity)

            self.get_logger().info(f'{command}')

            self._state_command = command

            # position gives relative position of box to centre of frame.

            self.get_logger().info(f'{results}')

        except Exception as e:
            self.get_logger().error(f'Error in on_step_callback: {e}')
