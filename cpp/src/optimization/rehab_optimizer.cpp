#include "biomech/rehab_optimizer.hpp"
#include <stdexcept>
#include <limits>

namespace biomech {
    RehabOptimizer::RehabOptimizer(const OptimizationConfig& config)
        : config_(config) {}

    void RehabOptimizer::setConstraints(const Constraints& constraints) {
        constraints_ = constraints;
    }

    std::vector<Joint> RehabOptimizer::optimizeTrajectory(
        const std::vector<Joint>& initial_pose,
        const std::vector<Joint>& target_pose,
        const std::function<double(const std::vector<Joint>&)>& cost_function
    ) {
        if (initial_pose.size() != target_pose.size()) {
            throw BiomechException("Initial and target poses must have the same size.");
        }

        std::vector<Joint> optimized_pose = initial_pose;
        double previous_cost = std::numeric_limits<double>::max();

        for (size_t iteration = 0; iteration < config_.max_iterations; ++iteration) {
            double current_cost = cost_function(optimized_pose);
            if (current_cost < config_.convergence_threshold) {
                break;
            }

            for (size_t i = 0; i < optimized_pose.size(); ++i) {
                double gradient = computeGradient(optimized_pose, i, cost_function);
                optimized_pose[i].position += -config_.constraint_tolerance * gradient;
            }

            if (std::abs(previous_cost - current_cost) < config_.convergence_threshold) {
                break;
            }

            previous_cost = current_cost;
        }

        return optimized_pose
    }

    std::vector<double> RehabOptimizer::optimizeMuscleActivations(
        const std::vector<Joint>& joints,
        const std::vector<Muscle>& muscles,
        const Eigen::VectorXd& target_forces
    ) {
        if (joints.size() != target_forces.size()) {
            throw BiomechException("Number of joints and target forces must match.");
        }

        std::vector<double> activations(muscles.size(), 0.0);
        for (size_t i = 0; i < muscles.size(); ++i) {
            activations[i] = target_forces[i] / muscles[i].max_force;
            activations[i] = std::clamp(activations[i], constraints_.min_muscle_activation, constraints_.max_muscle_activation);
        }

        return activations;
    }

    double RehabOptimizer::evaluateConstraints(const std::vector<Joint>& pose) {
        double constraint_violation = 0.0;

        for (const auto& joint : pose) {
            if (joint.min_angle != 0 && joint.max_angle != 0) {
                double angle_violation = std::max(0.0, joint.min_angle - joint.position.norm()) +
                                        std::max(0.0, joint.position.norm() - joint.max_angle);
                constraint_violation += angle_violation;
            }
        }

        return constraint_violation;
    }

    void RehabOptimizer::updateOptimizationParameters(double constraint_violation) {
        config_.constraint_tolerance = std::max(config_.constraint_tolerance * 0.9, 1e-8);
        config_.convergence_threshold = std::min(config_.convergence_threshold * 1.1, 1e-4);
    }

    double RehabOptimizer::computeGradient(
        const std::vector<Joint>& pose, size_t index,
        const std::function<double(const std::vector<Joint>&)>& cost_function
    ) {
        std::vector<Joint> perturbed_pose = pose;
        const double delta = 1e-6;
        perturbed_pose[index].position += Eigen::Vector3d(delta, delta, delta);
        double cost_plus = cost_function(perturbed_pose);

        perturbed_pose[index].position -= Eigen::Vector3d(2 * delta, 2 * delta, 2 * delta);
        double cost_minus = cost_function(perturbed_pose);

        return (cost_plus - cost_minus) / (2 * delta);
    }
} 
