#ifndef ROBOT_ARM_MOTION_PLANNER_HPP
#define ROBOT_ARM_MOTION_PLANNER_HPP

#include <kdl/trajectory.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/jntarray.hpp>
#include <vector>
#include <iostream>

#include <kdl/trajectory_segment.hpp>
#include <kdl/path_line.hpp>
#include <kdl/frames.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <memory>  // Pour std::unique_ptr

namespace robot_arm_motion_planner
{

class JointTrajectoryPlanner
{
public:
    static std::vector<std::vector<double>> interpolateJointMotion(
        const KDL::JntArray& q_start,
        const KDL::JntArray& q_end,
        const double v,
        const double a)
    {
        const unsigned int nj = q_start.rows();
        if (nj != q_end.rows()) {
            std::cerr << "Error: q_start and q_end must have same size.\n";
            return {};
        }

        std::vector<KDL::VelocityProfile_Trap> vel_profiles(nj);
        std::vector<double> durations(nj);

        for (unsigned int i = 0; i < nj; ++i) {
            vel_profiles[i].SetMax(v, a);
            vel_profiles[i].SetProfile(q_start(i), q_end(i));
            durations[i] = vel_profiles[i].Duration();
        }

        double total_duration = *std::max_element(durations.begin(), durations.end());
        // std::cout << "Total trajectory duration: " << total_duration << " s" << std::endl;

        if (total_duration <= 0.0) {
            std::cerr << "Error: total_duration <= 0.0\n";
            return {};
        }

        std::vector<std::vector<double>> trajectory;
        const int max_points = 10000;
        int point_count = 0;

        for (double t = 0.0; t <= total_duration && point_count < max_points; t += 0.01, ++point_count) { //Time step: 0.01
            std::vector<double> positions, velocities, accelerations;

            for (unsigned int i = 0; i < nj; ++i) {
                double t_i = std::min(t, vel_profiles[i].Duration());
                positions.push_back(vel_profiles[i].Pos(t_i));
                velocities.push_back(vel_profiles[i].Vel(t_i));
                accelerations.push_back(vel_profiles[i].Acc(t_i));
            }

            trajectory.push_back(positions);
            trajectory.push_back(velocities);
            trajectory.push_back(accelerations);
        }

        if (point_count == max_points) {
            std::cerr << "Warning: Trajectory point limit reached (" << max_points << ").\n";
        }

        return trajectory;
    }


    


    static KDL::Trajectory* GenerateCartesianTrajectory(const KDL::Frame& pose1,
    const KDL::Frame& pose2,
    double linear_vel,
    double linear_acc) {
        using namespace KDL;

        double eq_radius = 0.01;  // tolerance for path equality

        auto* rot_interp = new RotationalInterpolation_SingleAxis();
        auto* path = new Path_Line(pose1, pose2, rot_interp, eq_radius);

        auto* vel_profile = new VelocityProfile_Trap(linear_vel, linear_acc);
        vel_profile->SetProfile(0, path->PathLength());

        return new Trajectory_Segment(path, vel_profile);
    }

        


};

}  // namespace robot_arm_motion_planner

#endif // ROBOT_ARM_MOTION_PLANNER_HPP
