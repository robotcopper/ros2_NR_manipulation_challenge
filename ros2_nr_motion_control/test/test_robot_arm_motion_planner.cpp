#include <gtest/gtest.h>
#include "robot_arm_motion_planner/robot_arm_motion_planner.hpp"
#include <kdl/frames.hpp>

using namespace robot_arm_motion_planner;
using namespace KDL;

// Cartesian path generation test
TEST(CartesianTrajectoryTest, GeneratesCorrectStartAndEnd) {
    Frame pose1 = Frame(Rotation::RPY(0, 0, 0), Vector(0.0, 0.0, 0.0));
    Frame pose2 = Frame(Rotation::RPY(0, 0, 0), Vector(1.0, 0.0, 0.0));

    double v = 0.5;
    double a = 0.2;

    std::unique_ptr<Trajectory> traj(JointTrajectoryPlanner::GenerateCartesianTrajectory(pose1, pose2, v, a));

    ASSERT_TRUE(traj != nullptr);
    EXPECT_NEAR(traj->Pos(0.0).p.x(), 0.0, 1e-5);
    EXPECT_NEAR(traj->Pos(traj->Duration()).p.x(), 1.0, 1e-5);
}

// Joint space path generation test
TEST(JointMotionInterpolationTest, ReturnsNonEmptyTrajectory) {
    JntArray q_start(3), q_end(3);
    q_start(0) = 0.0; q_start(1) = 0.0; q_start(2) = 0.0;
    q_end(0) = 1.0; q_end(1) = -1.0; q_end(2) = 0.5;

    double v = 1.0;
    double a = 0.5;

    auto trajectory = JointTrajectoryPlanner::interpolateJointMotion(q_start, q_end, v, a);

    ASSERT_FALSE(trajectory.empty());
    // Check that the first position corresponds to the starting point
    EXPECT_NEAR(trajectory[0][0], 0.0, 1e-5);
    EXPECT_NEAR(trajectory[0][1], 0.0, 1e-5);
    EXPECT_NEAR(trajectory[0][2], 0.0, 1e-5);

    // Checks that the last position corresponds to the end point
    auto final_pos = trajectory[trajectory.size() - 3];  // positions before final velocities and accels
    EXPECT_NEAR(final_pos[0], 1.0, 1e-1);
    EXPECT_NEAR(final_pos[1], -1.0, 1e-1);
    EXPECT_NEAR(final_pos[2], 0.5, 1e-1);
}
