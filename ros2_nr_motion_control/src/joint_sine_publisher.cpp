#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

class TrajectoryPointPublisher : public rclcpp::Node
{
public:
    TrajectoryPointPublisher()
    : Node("trajectory_point_publisher")
    {
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TrajectoryPointPublisher::jointStateCallback, this, std::placeholders::_1));

        point_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
            "/target_joint_positions", 10);

        timer_ = this->create_wall_timer(10ms, std::bind(&TrajectoryPointPublisher::publishTarget, this));

        start_time_ = this->now();
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (joint_names_.empty() && !msg->name.empty()) {
            joint_names_ = msg->name;
            RCLCPP_INFO(this->get_logger(), "\033[1;32mJoint names received.\033[0m \n Ready to publish target positions!");
        }
    }

    void publishTarget()
    {
        if (joint_names_.empty()) return;

        rclcpp::Duration elapsed = this->now() - start_time_;
        double t = elapsed.seconds();

        trajectory_msgs::msg::JointTrajectoryPoint point_msg;
        point_msg.positions.resize(joint_names_.size());

        for (size_t i = 0; i < joint_names_.size(); ++i) {
            point_msg.positions[i] = offset_ + amplitude_ * std::sin(2 * M_PI * frequency_ * t);
        }

        point_msg.time_from_start = rclcpp::Duration::from_seconds(0.01);
        point_pub_->publish(point_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr point_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> joint_names_;
    rclcpp::Time start_time_;
    double frequency_ = 0.2; // Hz
    double amplitude_ = -0.5; // radians
    double offset_ = -M_PI / 2.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPointPublisher>());
    rclcpp::shutdown();
    return 0;
}
