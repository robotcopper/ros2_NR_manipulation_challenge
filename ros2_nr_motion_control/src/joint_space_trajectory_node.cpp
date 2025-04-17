#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "robot_arm_motion_planner/robot_arm_motion_planner.hpp"
#include <kdl/jntarray.hpp>
#include <vector>

using namespace std::chrono_literals;

class JointTrajectoryPublisher : public rclcpp::Node
{
public:
    JointTrajectoryPublisher()
        : Node("joint_trajectory_publisher"),
          q_start_kdl(6), 
          q_end_kdl(6),
          current_point_index_(0)
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
            "/target_joint_positions", 10);

        double q_start[6] = {3.0, -1.0, 0.0, -1.0, 1.0, 0.0}; // Point1
        double q_end[6]   = {3.14, -3.0, 2.0, 1.0, -1.0, 3.0}; // Point2
        v = 1.0; // joints velocity
        a = 1.0; // joints acceleration


        for (int i = 0; i < 6; ++i) {
            q_start_kdl(i) = q_start[i];
            q_end_kdl(i) = q_end[i];
        }

        generateTrajectory();

        // Calling publishNextPoint each 0.01
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.01),
            std::bind(&JointTrajectoryPublisher::publishNextPoint, this));
    }

private:
    void generateTrajectory()
    {
        trajectory_data_ = robot_arm_motion_planner::JointTrajectoryPlanner::interpolateJointMotion(
            q_start_kdl, q_end_kdl, v, a);

        current_point_index_ = 0;
        RCLCPP_INFO(this->get_logger(), "New trajectory generated.");
    }

    void publishNextPoint()
    {
        if (trajectory_data_.empty() || current_point_index_ * 3 + 2 >= trajectory_data_.size()) {
            // Switching points
            std::swap(q_start_kdl, q_end_kdl);
            generateTrajectory();
            return;
        }

        trajectory_msgs::msg::JointTrajectoryPoint point;

        const auto &positions = trajectory_data_[current_point_index_ * 3];
        const auto &velocities = trajectory_data_[current_point_index_ * 3 + 1];
        const auto &accelerations = trajectory_data_[current_point_index_ * 3 + 2];

        point.positions = positions;
        point.velocities = velocities;
        point.accelerations = accelerations;
        point.time_from_start = rclcpp::Duration::from_seconds(current_point_index_ * 0.01); // Elapsed time

        publisher_->publish(point);

        current_point_index_++;
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    KDL::JntArray q_start_kdl, q_end_kdl;
    std::vector<std::vector<double>> trajectory_data_;
    size_t current_point_index_;

    double v;
    double a;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointTrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
