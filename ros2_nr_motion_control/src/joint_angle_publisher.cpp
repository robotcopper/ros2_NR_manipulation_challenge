#include <rclcpp/rclcpp.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

class JointTrajectoryActionClient : public rclcpp::Node
{
public:
    JointTrajectoryActionClient()
    : Node("joint_trajectory_action_client"), action_client_(nullptr), goal_active_(false)
    {
        action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            this, "/joint_trajectory_controller/follow_joint_trajectory");

        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&JointTrajectoryActionClient::jointStateCallback, this, std::placeholders::_1));
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (joint_names_.empty() && !msg->name.empty()) {
            joint_names_ = msg->name;
            RCLCPP_INFO(this->get_logger(), "\033[1;32mJoint names received.\033[0m \n Ready to move!");
            sendTrajectory();
        }
    }

    bool has_goal_changed(const std::vector<double>& new_goal)
    {
        if (last_sent_positions_.size() != new_goal.size()) return true;
        for (size_t i = 0; i < new_goal.size(); ++i)
        {
            if (std::abs(last_sent_positions_[i] - new_goal[i]) > 1e-3)
                return true;
        }
        return false;
    }

    void sendTrajectory()
    {
        if (joint_names_.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for joint names...");
            return;
        }

        if (!action_client_->wait_for_action_server(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
            return;
        }

        if (goal_active_) {
            RCLCPP_DEBUG(this->get_logger(), "A goal is still active, skipping this cycle.");
            return;
        }

        // Setpoint example
        std::vector<double> target_positions = {0.0, -1.0, 1.5, 0.0, 1.0, 0.0};

        if (!has_goal_changed(target_positions)) {
            RCLCPP_DEBUG(this->get_logger(), "Target unchanged. No new goal sent.");
            return;
        }

        auto goal = control_msgs::action::FollowJointTrajectory::Goal();
        goal.trajectory.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = target_positions;
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        goal.trajectory.points.push_back(point);

        auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&JointTrajectoryActionClient::resultCallback, this, std::placeholders::_1);

        goal_active_ = true;
        last_sent_positions_ = target_positions;
        action_client_->async_send_goal(goal, send_goal_options);
        // RCLCPP_INFO(this->get_logger(), "Goal sent to controller.");
    }

    void resultCallback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
    {
        goal_active_ = false;

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            // RCLCPP_INFO(this->get_logger(), "Trajectory successfully followed.");
        } else {
            // RCLCPP_ERROR(this->get_logger(), "Failed to follow trajectory.");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> joint_names_;
    std::vector<double> last_sent_positions_;
    bool goal_active_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointTrajectoryActionClient>());
    rclcpp::shutdown();
    return 0;
}
