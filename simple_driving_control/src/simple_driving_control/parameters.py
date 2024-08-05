from dataclasses import dataclass


@dataclass
class VehicleParams:
    wheelbase: float


@dataclass
class ControllerParams:
    accel_k_p: float
    accel_k_d: float
