#include <gtest/gtest.h>
#include "biomech/rehab_optimizer.hpp"

TEST(OptimizationTest, OptimizeTrajectory) {
    OptimizationConfig config = {100, 1e-6, 0.01};
    RehabOptimizer optimizer(config);

    std::vector<Joint> initial_pose = {
        {Eigen::Vector3d(0, 0, 0)},
        {Eigen::Vector3d(1, 0, 0)},
    };
    std::vector<Joint> target_pose = {
        {Eigen::Vector3d(0, 1, 0)},
        {Eigen::Vector3d(1, 1, 0)},
    };

    auto cost_function = [](const std::vector<Joint>& pose) {
        double cost = 0.0;
        for (const auto& joint : pose) {
            cost += joint.position.squaredNorm();
        }
        return cost;
    };

    std::vector<Joint> optimized_pose = optimizer.optimizeTrajectory(initial_pose, target_pose, cost_function);

    EXPECT_NEAR(optimized_pose[0].position.x(), 0.0, 1e-3);
    EXPECT_NEAR(optimized_pose[0].position.y(), 1.0, 1e-3);
    EXPECT_NEAR(optimized_pose[1].position.x(), 1.0, 1e-3);
    EXPECT_NEAR(optimized_pose[1].position.y(), 1.0, 1e-3);
}

TEST(OptimizationTest, MuscleActivationOptimization) {
    OptimizationConfig config = {100, 1e-6, 0.01};
    RehabOptimizer optimizer(config);

    std::vector<Joint> joints = {
        {Eigen::Vector3d(0, 0, 0)},
        {Eigen::Vector3d(1, 0, 0)},
    };
    std::vector<Muscle> muscles = {
        {100.0},
        {150.0},
    };
    Eigen::VectorXd target_forces(2);
    target_forces << 50.0, 75.0;

    std::vector<double> activations = optimizer.optimizeMuscleActivations(joints, muscles, target_forces);

    EXPECT_NEAR(activations[0], 0.5, 1e-3);
    EXPECT_NEAR(activations[1], 0.5, 1e-3);
}
