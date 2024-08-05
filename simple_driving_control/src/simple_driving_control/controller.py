import numpy as np

from simple_driving_control.parameters import VehicleParams, ControllerParams
from simple_driving_control.vehicle_state import VehicleState


class VehicleController:
    def __init__(self, 
                 state: VehicleState,
                 vehicle_params: VehicleParams, 
                 controller_params: ControllerParams):
        self.state = state
        self.vehicle_params = vehicle_params
        self.controller_params = controller_params

        self.e_last = 0

    def commands(self, target: np.array):
        xy: np.array = self.state.get_xy()
        yaw: float = self.state.get_theta()

        # Transform target point to vehicle reference frame.
        xy_trans = np.array([
            [np.cos(yaw), np.sin(yaw)],
            [-np.sin(yaw), np.cos(yaw)],
        ]) @ (target - xy)

        # Calculate steer.
        # Get angle between vehicle and target point.
        alpha = np.arctan2(xy_trans[0], xy_trans[1])

        # Compute pure pursuit angle.
        steer = np.arctan2(2 * self.vehicle_params.wheelbase * np.sin(alpha), 
                           np.linalg.norm(xy_trans))
        
        # Calculate throttle
        e_sign = 1 if (xy_trans[0] > 0) else -1
        e = np.linalg.norm(self.state.get_xy() - target) * e_sign
        
        prop = e * self.controller_params.accel_k_p
        deriv = (e - self.e_last) * self.controller_params.accel_k_d

        self.e_last = e

        throttle = prop + deriv
        
        return np.array([throttle, steer])
