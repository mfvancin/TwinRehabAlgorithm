#pragma once
#include "types.hpp"
#include <functional>

namespace biomech {

class RehabOptimizer {
    public:
        struct OptimizationConfig {
            size_t max_iterations{1000};
            double convergence_threshold{1e-6};
            double constraint_tolerance{1e-4};
            bool use_muscle_dynamics{true};
        };

        struct Constraints {
            std::vector<std::pair<double, double>> joint_limits;
            std::vector<double> max_velocities;
            std::vector<double> max_accelerations;
            double min_muscle_activation{0.01};
            double max_muscle_activation{1.0};
        };

        explicit RehabOptimizer(const OptimizationConfig& config = OptimizationConfig{});

        void setConstraints(const Constraints& constraints);

        std::vector<Joint> optimizeTrajectory(
            const std::vector<Joint>& initial_pose,
            const std::vector<Joint>& target_pose,
            const std::function<double(const std::vector<Joint>&)>& cost_function
        );

        std::vector<double> optimizeMuscleActivations(
            const std::vector<Joint>& joints,
            const std::vector<Muscle>& muscles,
            const Eigen::VectorXd& target_forces
        );

    private:
        OptimizationConfig config_;
        Constraints constraints_;
        
        double evaluateConstraints(const std::vector<Joint>& pose);
        void updateOptimizationParameters(double constraint_violation);
    };
}