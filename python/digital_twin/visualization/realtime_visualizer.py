import matplotlib.pyplot as plt

class RealtimeVisualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots()

    def visualize(self, joint_angles, time_stamps):
        self.ax.clear()
        self.ax.plot(time_stamps, joint_angles, label="Joint Angles")
        self.ax.set_title("Real-Time Joint Angle Visualization")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Joint Angle (degrees)")
        plt.draw()
        plt.pause(0.01)
