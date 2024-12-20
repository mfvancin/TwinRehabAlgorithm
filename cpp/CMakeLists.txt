cmake_minimum_required(VERSION 3.15)

# Set compiler flags
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Werror -pedantic)
endif()

# Add include directories
include_directories(${PROJECT_SOURCE_DIR}/cpp/include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Create library
add_library(biomech_core
    src/core/musculoskeletal_model.cpp
    src/analysis/kinematics_solver.cpp
    src/analysis/motion_analyzer.cpp
    src/optimization/rehab_optimizer.cpp
)

target_link_libraries(biomech_core
    PUBLIC
        Eigen3::Eigen
        ${VTK_LIBRARIES}
        OpenMP::OpenMP_CXX
)