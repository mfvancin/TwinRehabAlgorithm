set -e

CPP_DIR="./bindings"
PYTHON_DIR="./python"

echo "Building C++ bindings..."

cd $CPP_DIR
make clean  
make       

echo "C++ bindings built successfully."

echo "Building Python bindings..."

cp build/bindings.so $PYTHON_DIR/

echo "Python bindings built successfully."

echo "Build process completed!"
