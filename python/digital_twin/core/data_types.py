class MovementData:
    def __init__(self, joint_angles, time_stamps):
        self.joint_angles = joint_angles
        self.time_stamps = time_stamps

    def get_movement(self):
        return self.joint_angles, self.time_stamps
