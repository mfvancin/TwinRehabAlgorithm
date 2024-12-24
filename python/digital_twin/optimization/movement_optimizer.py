class MovementOptimizer:
    def __init__(self, target_movement, current_movement):
        self.target_movement = target_movement
        self.current_movement = current_movement

    def optimize(self):

        print("Optimizing movement...")
        difference = self.target_movement - self.current_movement
        return self.current_movement + 0.1 * difference
