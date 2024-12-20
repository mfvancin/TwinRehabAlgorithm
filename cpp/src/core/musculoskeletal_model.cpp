#include "biomech/types.hpp"
#include <stdexcept>

namespace biomech {

class MusculoskeletalModel {
    public:
        MusculoskeletalModel(const std::vector<Joint>& joints, const std::vector<Segment>& segments)
            : joints_(joints), segments_(segments) {}

        void addJoint(const Joint& joint) {
            joints_.push_back(joint);
        }

        void addSegment(const Segment& segment) {
            segments_.push_back(segment);
        }

        const std::vector<Joint>& getJoints() const {
            return joints_;
        }

        const std::vector<Segment>& getSegments() const {
            return segments_;
        }

        Eigen::Vector3d computeCenterOfMass() const {
            Eigen::Vector3d total_mass_center = Eigen::Vector3d::Zero();
            double total_mass = 0.0;

            for (const auto& segment : segments_) {
                total_mass_center += segment.mass * segment.center_of_mass;
                total_mass += segment.mass;
            }

            if (total_mass == 0.0) {
                throw std::runtime_error("Total mass is zero. Cannot compute center of mass.");
            }

            return total_mass_center / total_mass;
        }

    private:
        std::vector<Joint> joints_;
        std::vector<Segment> segments_;
    };
} 
