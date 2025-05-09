#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include "robot_arm_motion_planner/robot_arm_motion_planner.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using std::placeholders::_1;

class InverseKinematicsNode : public rclcpp::Node {
public:
    InverseKinematicsNode() : Node("inverse_kinematics_node") {
        RCLCPP_INFO(this->get_logger(), "Waiting for /robot_description...");
        urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
            "robot_description",
            rclcpp::QoS(rclcpp::KeepLast(1))
              .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
            std::bind(&InverseKinematicsNode::on_urdf_received, this, _1));

        pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
            "/target_joint_positions", 10
        );

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/ik_trajectory_markers", 10);
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    KDL::Chain kdl_chain_;
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;

    void on_urdf_received(const std_msgs::msg::String::SharedPtr msg) {
        if (ik_solver_) return;

        urdf::Model model;
        if (!model.initString(msg->data)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF.");
            return;
        }

        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL tree.");
            return;
        }

        auto first_joint = model.joints_.begin();  // First joint
        auto last_joint = --model.joints_.end();   // Last joint
        RCLCPP_INFO(this->get_logger(), "First joint: %s", first_joint->second->name.c_str());
        RCLCPP_INFO(this->get_logger(), "Last joint: %s", last_joint->second->name.c_str());
        std::shared_ptr<const urdf::Link> first_parent_link = model.getLink(first_joint->second->parent_link_name);
        std::shared_ptr<const urdf::Link> last_child_link = model.getLink(last_joint->second->parent_link_name);
        std::string first_link = first_parent_link->name;  // First link
        std::string last_link = last_child_link->name;    // Last link
        RCLCPP_INFO(this->get_logger(), "First link: %s", first_link.c_str());
        RCLCPP_INFO(this->get_logger(), "Last link: %s", last_link.c_str());

        if (!kdl_tree.getChain(first_link, last_link, kdl_chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get chain from %s to %s", first_link.c_str(), last_link.c_str());
            return;
        }

        ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(kdl_chain_);
        RCLCPP_INFO(this->get_logger(), "URDF parsed and IK solver initialized.");

        this->solveIK();
    }

    KDL::Trajectory* traj_;
    double t_;
    KDL::JntArray q_init_;
    rclcpp::TimerBase::SharedPtr timer_;
    KDL::Frame start_pose_;
    KDL::Frame end_pose_;
    double linear_vel_;
    double linear_acc_;

    void solveIK() {
        if (!ik_solver_) return;

        /// ############# Settings ############# //

        start_pose_ = KDL::Frame(KDL::Rotation::RPY(0.0, M_PI / 3, 0.0), KDL::Vector(0.5, 0.5, 0.25)); // Pose1
        end_pose_ = KDL::Frame(KDL::Rotation::RPY(0.0, -M_PI / 2, 0.0), KDL::Vector(-0.25, 0.25, 0.75)); // Pose2
        linear_vel_ = 0.2; // linear velocity
        linear_acc_ = 0.1; // linear acceleration

        /// ############# Settings ############# //

        traj_ = robot_arm_motion_planner::JointTrajectoryPlanner::GenerateCartesianTrajectory(start_pose_, end_pose_, linear_vel_, linear_acc_);
        
        t_ = 0.0;
        q_init_ = KDL::JntArray(kdl_chain_.getNrOfJoints());
        for (unsigned int i = 0; i < q_init_.rows(); ++i)
            q_init_(i) = 0.0;
    
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.01),
            std::bind(&InverseKinematicsNode::publishNextPoint, this)
        );

        publishTrajectoryMarkers();
    
        RCLCPP_INFO(this->get_logger(), "Started IK trajectory timer.");
    }

    void publishNextPoint() {
        if (t_ > traj_->Duration()) {
            delete traj_;
            std::swap(start_pose_, end_pose_);
            traj_ = robot_arm_motion_planner::JointTrajectoryPlanner::GenerateCartesianTrajectory(start_pose_, end_pose_, linear_vel_, linear_acc_);
            t_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Swapped start and end. Restarting trajectory.");
            return;
        }
    
        KDL::Frame pose = traj_->Pos(t_);
        KDL::JntArray q_sol(kdl_chain_.getNrOfJoints());
        int ret = ik_solver_->CartToJnt(q_init_, pose, q_sol);
    
        if (ret >= 0) {
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions.resize(q_sol.rows());
            for (unsigned int i = 0; i < q_sol.rows(); ++i)
                pt.positions[i] = q_sol(i);
            pt.time_from_start = rclcpp::Duration::from_seconds(t_);
            pub_->publish(pt);
            // RCLCPP_INFO(this->get_logger(), "Published point at t=%.2f", t_);
    
            q_init_ = q_sol;
        } else {
            RCLCPP_WARN(this->get_logger(), "IK failed at t=%.2f", t_);
        }
    
        t_ += 0.01;
    }

    void publishTrajectoryMarkers() {
        if (!traj_) return;
    
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
    
        for (double t = 0.0; t <= traj_->Duration(); t += 0.05) {
            KDL::Frame pose = traj_->Pos(t);
    
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = this->now();
            marker.ns = "ik_trajectory";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = pose.p.x();
            marker.pose.position.y = pose.p.y();
            marker.pose.position.z = pose.p.z();
    
            double x, y, z, w;
            pose.M.GetQuaternion(x, y, z, w);
            marker.pose.orientation.x = x;
            marker.pose.orientation.y = y;
            marker.pose.orientation.z = z;
            marker.pose.orientation.w = w;
    
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
                  
            double duration = traj_->Duration();
            double normalized_t = t / duration;
            double mirrored_t = 2.0 * std::min(normalized_t, 1.0 - normalized_t);  
            double smooth_factor = 0.5 * (1.0 - std::cos(mirrored_t * M_PI)); 
            marker.color.a = 1.0;
            marker.color.r = 0.5 + 0.7 * smooth_factor;  
            marker.color.g = 0.0;
            marker.color.b = 0.0;  

            marker_array.markers.push_back(marker);
        }
    
        marker_pub_->publish(marker_array);
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InverseKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
