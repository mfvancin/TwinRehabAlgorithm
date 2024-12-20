import unittest
from bindings import bindings 

class TestBindings(unittest.TestCase):
    
    def test_model_interface(self):
        model = bindings.ModelInterface("path/to/model")
        model.load_model()
        result = model.predict([1.0, 2.0, 3.0])  
        self.assertIsNotNone(result)
    
    def test_kinematics_solver(self):
        solver = bindings.KinematicsSolver()
        solution = solver.solve([1.0, 2.0, 3.0])  
        self.assertIsNotNone(solution)
    
    def test_optimizer(self):
        optimizer = bindings.RehabOptimizer()
        optimized_data = optimizer.optimize([1.0, 2.0, 3.0]) 
        self.assertIsNotNone(optimized_data)

if __name__ == "__main__":
    unittest.main()
