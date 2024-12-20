#include "biomech/types.hpp"

namespace biomech {
    Joint::Joint(const std::string& name, double min_angle, double max_angle)
        : name(name), min_angle(min_angle), max_angle(max_angle) {}

    void Joint::updatePosition(const Eigen::Vector3d& position) {
        this->position = position;
    }

    void Joint::updateOrientation(const Eigen::Quaterniond& orientation) {
        this->orientation = orientation;
    }

    bool Joint::isWithinLimits(double angle) const {
        return angle >= min_angle && angle <= max_angle;
    }
} 
