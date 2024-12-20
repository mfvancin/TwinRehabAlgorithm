import numpy as np

class MotionProcessor:
    def __init__(self, data):
        self.data = data

    def filter_motion_data(self):
        print("Filtering motion data...")
        self.data.joint_angles = np.array(self.data.joint_angles)
        filtered_data = np.where(self.data.joint_angles > 0, self.data.joint_angles, 0)
        return filtered_data
