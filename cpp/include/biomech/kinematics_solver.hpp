#pragma once
#include "types.hpp"
#include <optional>

namespace biomech {

class KinematicsSolver {
    public:
        struct Config {
            double convergence_threshold{1e-6};
            size_t max_iterations{100};
            double damping_factor{0.1};
        };

        explicit KinematicsSolver(const Config& config = Config{});

        Eigen::VectorXd solveInverseKinematics(
            const std::vector<Joint>& target_poses,
            const Eigen::VectorXd& initial_guess
        );

        std::pair<Eigen::VectorXd, Eigen::VectorXd> solveInverseDynamics(
            const std::vector<Joint>& joints,
            const std::vector<Segment>& segments,
            const Eigen::VectorXd& external_forces
        );

        bool checkJointLimits(const std::vector<Joint>& joints) const;

    private:
        Config config_;
        Eigen::MatrixXd computeJacobian(const std::vector<Joint>& joints);
        void updateDampingFactor(double error);
    };
}