#ifndef DRIFT_CORRECTION_  // Publishers
rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;

// Timer
rclcpp::TimerBase::SharedPtr timer_;

// Callback functions
void tag_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
void control_loop();

// Current local position
double current_x_, current_y_, current_z_;
bool position_valid_{ false };

// Tag tracking state
bool tag_visible_{ false };
geometry_msgs::msg::PoseStamped::SharedPtr last_tag_pose_;

// Helper functions
void publish_correction_setpoint();
RIFT_CORRECTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <chrono>

class DriftCorrectionNode : public rclcpp::Node
{
public:
  DriftCorrectionNode();

private:
  // Parameters
  double desired_distance_;    // Desired distance from the rack (in meters)
  double correction_speed_;    // Speed of correction movement
  double distance_tolerance_;  // Tolerance for distance maintenance

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr forward_tag_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;

  // Publishers
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;

  // Callback functions
  void tag_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

  // Current local position
  double current_x_, current_y_, current_z_;
  bool position_valid_{ false };

  // Helper functions
  void publish_correction_setpoint(const geometry_msgs::msg::PoseStamped& tag_pose);
};

#endif  // DRIFT_CORRECTION_HPP
