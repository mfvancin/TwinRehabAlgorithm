cmake_minimum_required(VERSION 3.15)
project(musculoskeletal-rehab-twin VERSION 1.0.0 LANGUAGES CXX)

option(BUILD_TESTS "Build tests" ON)
option(BUILD_PYTHON_BINDINGS "Build Python bindings" ON)
option(BUILD_DOCUMENTATION "Build documentation" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenMP REQUIRED)

if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

add_subdirectory(cpp)
if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(bindings)
endif()

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(cpp/tests)
    if(BUILD_PYTHON_BINDINGS)
        add_subdirectory(bindings/tests)
    endif()
endif()

if(BUILD_DOCUMENTATION)
    add_subdirectory(docs)
endif()