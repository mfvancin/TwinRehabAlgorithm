import unittest
from digital_twin.core.model_interface import ModelInterface

class TestModelInterface(unittest.TestCase):
    def test_load_model(self):
        model = ModelInterface('path/to/model')
        model.load_model()  
        self.assertIsNotNone(model.model)

if __name__ == "__main__":
    unittest.main()
