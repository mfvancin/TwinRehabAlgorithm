import json

class PatientDatabase:
    def __init__(self, database_path):
        self.database_path = database_path

    def load_patient_data(self):
        with open(self.database_path, 'r') as f:
            data = json.load(f)
        print(f"Loaded patient data from {self.database_path}")
        return data
