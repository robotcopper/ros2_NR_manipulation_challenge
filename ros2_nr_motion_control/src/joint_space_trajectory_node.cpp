#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "robot_arm_motion_planner/robot_arm_motion_planner.hpp"
#include <kdl/jntarray.hpp>
#include <vector>

double q_start[6] = {3.0, -1.0, 0.0, -1.0, 1.0, 0.0};  // Point1
double q_end[6] = {3.14, -3.0, 2.0, 1.0, -1.0, 3.0};  // Point2
double v = 1.0;  // joints velocity
double a = 1.0;  // joints acceleration

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

        rclcpp::Rate rate(100);

        while (rclcpp::ok()) {
            std::vector<std::vector<double>> trajectory_data =
                robot_arm_motion_planner::JointTrajectoryPlanner::interpolateJointMotion(
                    q_start_kdl, q_end_kdl, v, a);

            for (size_t i = 0; i < trajectory_data.size(); i += 3) {
                trajectory_msgs::msg::JointTrajectoryPoint point;
                point.time_from_start = rclcpp::Duration::from_seconds(i / 3 * 0.01); // Elapsed time

                for (size_t j = 0; j < q_start_kdl.rows(); ++j) {
                    point.positions.push_back(trajectory_data[i][j]);
                    point.velocities.push_back(trajectory_data[i + 1][j]);
                    point.accelerations.push_back(trajectory_data[i + 2][j]);
                }

                publisher_->publish(point);
                rate.sleep();
            }

            // Switching points
            std::swap(q_start_kdl, q_end_kdl);
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
