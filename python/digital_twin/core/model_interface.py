class ModelInterface:
    def __init__(self, model_path):
        self.model_path = model_path
        self.model = None

    def load_model(self):
        print(f"Loading model from {self.model_path}")

    def predict(self, input_data):
        print("Making predictions...")
