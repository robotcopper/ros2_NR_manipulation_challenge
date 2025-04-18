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
          

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&InverseKinematicsNode::on_joint_states, this, _1)
        );

        pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
            "/target_joint_positions", 10
        );
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr pub_;

    std::vector<std::string> joint_names_;
    KDL::Chain kdl_chain_;
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;

    void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_names_ = msg->name;
    }

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

        std::string base_link = "base_link"; 
        std::string tip_link = "wrist_3_link"; 

        // auto first_joint = model.getJoint(joint_names_[0]);
        // auto last_joint = model.getJoint(joint_names_[5]);
        
        // std::string base_link = first_joint->parent_link_name;
        // std::string tip_link = last_joint->child_link_name;
        
        if (!kdl_tree.getChain(base_link, tip_link, kdl_chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get chain from %s to %s", base_link.c_str(), tip_link.c_str());
            return;
        }

        ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(kdl_chain_);
        RCLCPP_INFO(this->get_logger(), "URDF parsed and IK solver initialized.");

        this->solveIK();
    }

    void solveIK() {
        if (!ik_solver_) {
            RCLCPP_WARN(this->get_logger(), "IK solver not ready.");
            return;
        }

        KDL::JntArray q_init(kdl_chain_.getNrOfJoints());
        for (unsigned int i = 0; i < q_init.rows(); ++i)
            q_init(i) = 0.0;

        KDL::Frame target_pose(KDL::Rotation::Identity(), KDL::Vector(0.5, 0.5, 0.3));
        KDL::JntArray q_sol(kdl_chain_.getNrOfJoints());

        if (ik_solver_->CartToJnt(q_init, target_pose, q_sol) >= 0) {
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions.resize(q_sol.rows());
            for (unsigned int i = 0; i < q_sol.rows(); ++i)
                pt.positions[i] = q_sol(i);

            pub_->publish(pt);
            RCLCPP_INFO(this->get_logger(), "IK solution published.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "IK failed.");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InverseKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
