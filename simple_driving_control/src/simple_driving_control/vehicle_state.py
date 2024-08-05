import numpy as np


class VehicleState:
    def __init__(self, x, y, theta, v) -> None:
        self.state = np.array([x, y, theta, v])
    
    def set(self, state: np.array):
        self.state = state

    def get(self) -> np.array:
        return self.state
    
    def get_x(self) -> float:
        return self.state[0]
    
    def get_y(self) -> float:
        return self.state[1]
    
    def get_xy(self) -> np.array:
        return self.state[:2]
    
    def get_theta(self) -> float:
        return self.state[2]
    
    def get_v(self) -> float:
        return self.state[3]
    
    def set_xy(self, xy: np.array):
        self.state[:2] = xy

    def set_theta(self, theta: float):
        self.state[2] = theta

    def set_v(self, v: float):
        self.state[3] = v
