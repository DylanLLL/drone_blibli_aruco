#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

class SimplePrecisionLand : public rclcpp::Node
{
public:
  explicit SimplePrecisionLand();

private:
  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
  void publish_land_command();
  void control_loop();

  // Parameters
  double descent_velocity_;
  double hover_height_;
  double land_threshold_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;

  // Publishers
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  bool target_visible_{ false };
  bool is_landing_{ false };
  bool has_landed_{ false };
  geometry_msgs::msg::PoseStamped::SharedPtr last_target_pose_;
  rclcpp::Time last_target_time_;
};
