import unittest
import numpy as np
from digital_twin.analysis.motion_processor import MotionProcessor
from digital_twin.core.data_types import MovementData

class TestMotionProcessor(unittest.TestCase):
    def test_filter_motion_data(self):
        movement_data = MovementData([10, -5, 15, -1], [0, 1, 2, 3])
        processor = MotionProcessor(movement_data)
        filtered_data = processor.filter_motion_data()
        self.assertTrue(np.all(filtered_data >= 0))

if __name__ == "__main__":
    unittest.main()
