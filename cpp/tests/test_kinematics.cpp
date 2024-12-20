#include <gtest/gtest.h>
#include "biomech/kinematics.hpp"

TEST(KinematicsTest, ForwardKinematics) {
    std::vector<Joint> joints = {
        {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0)},
        {Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(1, 1, 0)},
    };

    Kinematics kinematics;
    Eigen::Vector3d end_effector = kinematics.forwardKinematics(joints);

    EXPECT_EQ(end_effector, Eigen::Vector3d(2, 1, 0));
}

TEST(KinematicsTest, InverseKinematics) {
    std::vector<Joint> joints(2);
    Eigen::Vector3d target = Eigen::Vector3d(2, 1, 0);

    Kinematics kinematics;
    bool success = kinematics.inverseKinematics(joints, target);

    EXPECT_TRUE(success);
    EXPECT_NEAR(joints[0].position.x(), 1.0, 1e-3);
    EXPECT_NEAR(joints[0].position.y(), 0.0, 1e-3);
    EXPECT_NEAR(joints[1].position.x(), 1.0, 1e-3);
    EXPECT_NEAR(joints[1].position.y(), 1.0, 1e-3);
}
