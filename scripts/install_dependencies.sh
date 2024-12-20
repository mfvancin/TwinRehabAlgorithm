set -e

echo "Installing Python dependencies..."

pip install -r requirements.txt

echo "Python dependencies installed successfully."

echo "Installing system dependencies..."

sudo apt-get update
sudo apt-get install -y build-essential cmake libpython3-dev

echo "System dependencies installed successfully."

echo "All dependencies installed!"
