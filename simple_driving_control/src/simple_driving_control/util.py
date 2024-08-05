import numpy as np


def quat_to_yaw(w, x, y, z):
    t1 = 2 * (w * z + x * y)
    t2 = 1 - 2 * (w * y - x * z)
    return 2 * np.arctan2(t1, t2)
