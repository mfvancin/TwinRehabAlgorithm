set -e

echo "Running Python tests..."

cd tests
python3 -m unittest discover -s . -p "test_*.py"  

echo "Python tests completed successfully."

echo "Running C++ tests..."

cd ../bindings
./run_cpp_tests.sh 

echo "C++ tests completed successfully."

echo "All tests passed!"
