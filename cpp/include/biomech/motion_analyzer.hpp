#pragma once
#include "types.hpp"
#include <map>

namespace biomech {
    class MotionAnalyzer {
    public:
        struct GaitParameters {
            double stride_length;
            double step_width;
            double cadence;
            std::map<std::string, std::vector<double>> joint_angles;
            std::map<std::string, std::vector<double>> joint_velocities;
            std::map<std::string, std::vector<double>> joint_accelerations;
        };

        struct AnalysisConfig {
            double sampling_rate{100.0};  
            bool filter_data{true};
            double cutoff_frequency{6.0};  
            size_t window_size{10};
        };

        explicit MotionAnalyzer(const AnalysisConfig& config = AnalysisConfig{});

        GaitParameters analyzeGait(const std::vector<std::vector<Joint>>& motion_sequence);
        
        std::vector<double> computeJointLoads(
            const std::vector<Joint>& joints,
            const std::vector<Segment>& segments,
            const Eigen::VectorXd& external_forces
        );

        std::map<std::string, double> computeSymmetryIndices(const GaitParameters& gait_params);

    private:
        AnalysisConfig config_;
        void filterData(std::vector<double>& data);
        std::vector<double> detectGaitEvents(const std::vector<std::vector<Joint>>& motion_sequence);
    };
}