#pragma once
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <string>
#include <cmath>

namespace biomech {
    class MusculoskeletalModel;
    class KinematicsSolver;
    class MotionAnalyzer;

    struct Joint {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::string name;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        Eigen::Vector3d linear_velocity;
        Eigen::Vector3d angular_velocity;
        Eigen::Vector3d linear_acceleration;
        Eigen::Vector3d angular_acceleration;
        
        double min_angle{0};
        double max_angle{0};
        double max_velocity{0};
        double max_torque{0};
        Joint(const std::string &name, double min_angle, double max_angle);
        void updatePosition(const Eigen::Vector3d &position);
        void updateOrientation(const Eigen::Quaterniond &orientation);
        bool isWithinLimits(double angle) const;
    };

    struct Segment {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::string name;
        double mass;
        Eigen::Vector3d center_of_mass;
        Eigen::Matrix3d inertia_tensor;
        std::vector<size_t> connected_joints;
        Eigen::Vector3d dimensions;  
    };

    struct Muscle {
        std::string name;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> attachment_points;
        double max_force;
        double optimal_fiber_length;
        double tendon_slack_length;
        double pennation_angle;
    };

    class BiomechException : public std::runtime_error {
    public:
        explicit BiomechException(const std::string& msg) : std::runtime_error(msg) {}
    };
} 