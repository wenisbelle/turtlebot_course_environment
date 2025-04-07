#include <functional>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ignition/math/Vector3.hh>

class DifferentialPlugin : public gazebo::ModelPlugin {

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// Joint names
  std::string joint_a_name_, joint_b_name_;
  std::string joint_wheel_FL_name_, joint_wheel_RL_name_;
  std::string joint_wheel_FR_name_, joint_wheel_RR_name_;

  /// Force multiplier constant
  double force_constant_;

  /// Pointers to joints
  gazebo::physics::JointPtr joint_a_, joint_b_;
  gazebo::physics::JointPtr joint_wheel_FL_, joint_wheel_RL_;
  gazebo::physics::JointPtr joint_wheel_FR_, joint_wheel_RR_;

  /// Pointer to the model
  gazebo::physics::ModelPtr model_;

  /// Connects to physics update event
  gazebo::event::ConnectionPtr update_connection_;

  /// TF2 broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Create ros_node configured from sdf
    ros_node_ = gazebo_ros::Node::Get(_sdf);

    model_ = _parent;

    // Load joint names and force constant
    if (!_sdf->HasElement("jointA") || !_sdf->HasElement("jointB") ||
        !_sdf->HasElement("jointWheelFL") || !_sdf->HasElement("jointWheelRL") ||
        !_sdf->HasElement("jointWheelFR") || !_sdf->HasElement("jointWheelRR") ||
        !_sdf->HasElement("forceConstant")) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Missing required elements in SDF. DifferentialPlugin could not be loaded.");
      return;
    }

    joint_a_name_ = _sdf->GetElement("jointA")->Get<std::string>();
    joint_b_name_ = _sdf->GetElement("jointB")->Get<std::string>();
    joint_wheel_FL_name_ = _sdf->GetElement("jointWheelFL")->Get<std::string>();
    joint_wheel_RL_name_ = _sdf->GetElement("jointWheelRL")->Get<std::string>();
    joint_wheel_FR_name_ = _sdf->GetElement("jointWheelFR")->Get<std::string>();
    joint_wheel_RR_name_ = _sdf->GetElement("jointWheelRR")->Get<std::string>();
    force_constant_ = _sdf->GetElement("forceConstant")->Get<double>();

    // Get joints
    joint_a_ = model_->GetJoint(joint_a_name_);
    joint_b_ = model_->GetJoint(joint_b_name_);
    joint_wheel_FL_ = model_->GetJoint(joint_wheel_FL_name_);
    joint_wheel_RL_ = model_->GetJoint(joint_wheel_RL_name_);
    joint_wheel_FR_ = model_->GetJoint(joint_wheel_FR_name_);
    joint_wheel_RR_ = model_->GetJoint(joint_wheel_RR_name_);

    if (!joint_a_ || !joint_b_ || !joint_wheel_FL_ || !joint_wheel_RL_ || !joint_wheel_FR_ || !joint_wheel_RR_) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Could not find one or more joints. DifferentialPlugin could not be loaded.");
      return;
    }

    // Initialize the TF2 broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);

    // Connect to world update event
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&DifferentialPlugin::OnUpdate, this));

    RCLCPP_INFO_STREAM(ros_node_->get_logger(),
                       "DifferentialPlugin loaded! Joint A: \""
                           << joint_a_name_ << "\", Joint B: \""
                           << joint_b_name_
                           << "\", Force Constant: " << force_constant_);
  }

  void OnUpdate() {
    double angle_diff = joint_a_->Position() - joint_b_->Position();
    joint_a_->SetForce(0, -angle_diff * force_constant_);
    joint_b_->SetForce(0, angle_diff * force_constant_);

    // Publish TF transforms for rockers and wheels
    PublishJointTF(joint_a_, "base_footprint"); // For rocker_R_link
    PublishJointTF(joint_b_, "base_footprint"); // For rocker_L_link

    // For left wheels (invert rotation)
    PublishWheelTF(joint_wheel_FL_, joint_a_->GetChild()->GetName(), -0.152645, -0.085316, -0.088254, true); // wheel_FL_link to rocker_L_link
    PublishWheelTF(joint_wheel_RL_, joint_a_->GetChild()->GetName(), 0.152645, -0.085316, -0.088254, true);  // wheel_RL_link to rocker_L_link

    // For right wheels (normal rotation)
    PublishWheelTF(joint_wheel_FR_, joint_b_->GetChild()->GetName(), 0.152645, -0.085316, -0.088254, false); // wheel_FR_link to rocker_R_link
    PublishWheelTF(joint_wheel_RR_, joint_b_->GetChild()->GetName(), -0.152645, -0.085316, -0.088254, false); // wheel_RR_link to rocker_R_link
  }

  void PublishJointTF(gazebo::physics::JointPtr joint, const std::string &parent_frame) {
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = ros_node_->now();
    transformStamped.header.frame_id = parent_frame;

    // Get the child link of the joint
    auto child_link = joint->GetChild();
    std::string child_frame = child_link->GetName();
    transformStamped.child_frame_id = child_frame;

    // Get the pose of the link relative to the model
    auto pose = child_link->RelativePose();

    // Here we keep the full transform for the rocker joints
    transformStamped.transform.translation.x = pose.Pos().X();
    transformStamped.transform.translation.y = pose.Pos().Y();
    transformStamped.transform.translation.z = pose.Pos().Z();
    transformStamped.transform.rotation.x = pose.Rot().X();
    transformStamped.transform.rotation.y = pose.Rot().Y();
    transformStamped.transform.rotation.z = pose.Rot().Z();
    transformStamped.transform.rotation.w = pose.Rot().W();

    // Broadcast the transform
    tf_broadcaster_->sendTransform(transformStamped);
  }

  void PublishWheelTF(gazebo::physics::JointPtr joint, const std::string &parent_frame, double x, double y, double z, bool invert_rotation) {
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = ros_node_->now();
    transformStamped.header.frame_id = parent_frame;

    // Set the hardcoded fixed position of the wheel relative to the rocker
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;

    // Retrieve the joint angle
    double joint_angle = joint->Position(0);

    // Invert rotation for left wheels
    if (invert_rotation) {
      joint_angle = -joint_angle;
    }

    // Assuming rotation around the Y-axis for the wheel
    ignition::math::Quaterniond wheel_rotation(0, joint_angle, 0);

    transformStamped.transform.rotation.x = wheel_rotation.X();
    transformStamped.transform.rotation.y = wheel_rotation.Y();
    transformStamped.transform.rotation.z = wheel_rotation.Z();
    transformStamped.transform.rotation.w = wheel_rotation.W();

    // Get the child link of the joint
    auto child_link = joint->GetChild();
    std::string child_frame = child_link->GetName();
    transformStamped.child_frame_id = child_frame;

    // Broadcast the transform
    tf_broadcaster_->sendTransform(transformStamped);
  }
};

GZ_REGISTER_MODEL_PLUGIN(DifferentialPlugin)
