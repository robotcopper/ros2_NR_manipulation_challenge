#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "robot_arm_motion_planner/robot_arm_motion_planner.hpp"
#include <kdl/jntarray.hpp>
#include <vector>

double q_start[6] = {3.0, -1.0, 0.0, -1.0, 1.0, 0.0};  // Point1
double q_end[6] = {3.14, -3.0, 2.0, 1.0, -1.0, 3.0};  // Point2
double v_max = 1.0;  // joints velocity
double a_max = 1.0;  // joints acceleration

class JointTrajectoryPublisher : public rclcpp::Node
{
public:
    JointTrajectoryPublisher()
        : Node("joint_trajectory_publisher")
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("/target_joint_positions", 10);

        KDL::JntArray q_start_kdl(6), q_end_kdl(6);
        for (int i = 0; i < 6; ++i) {
            q_start_kdl(i) = q_start[i];
            q_end_kdl(i) = q_end[i];
        }

        // Generate trajectory
        std::vector<std::vector<double>> trajectory_data =
            robot_arm_motion_planner::JointTrajectoryPlanner::interpolateJointMotion(
                q_start_kdl, q_end_kdl, v_max, a_max);

        rclcpp::Time last_publish_time = this->get_clock()->now();

        for (size_t i = 0; i < trajectory_data.size(); i += 3) {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.time_from_start = rclcpp::Duration::from_seconds(i * 0.01);  // Elapsed time

            for (size_t j = 0; j < q_start_kdl.rows(); ++j) {
                point.positions.push_back(trajectory_data[i][j]);
                point.velocities.push_back(trajectory_data[i + 1][j]);
                point.accelerations.push_back(trajectory_data[i + 2][j]);
            }

            // Waiting time step before publishing
            rclcpp::Duration wait_time = rclcpp::Duration::from_seconds(0.01);
            rclcpp::sleep_for(std::chrono::nanoseconds(wait_time.nanoseconds()));
            
            publisher_->publish(point);

            last_publish_time = this->get_clock()->now();
        }
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointTrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
