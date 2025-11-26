#ifndef DRIFT_CORRECTION_HPP
#define DRIFT_CORRECTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <chrono>

class DriftCorrection : public rclcpp::Node
{
public:
  DriftCorrection();

private:
  // Parameters
  double desired_distance_;    // Desired distance from the rack (in meters)
  double correction_speed_;    // Speed of correction movement
  double distance_tolerance_;  // Tolerance for distance maintenance

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;

  // Publishers
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Flight state management
  enum class FlightState
  {
    INIT,
    ARMING,
    OFFBOARD_READY,
    HOVERING,
    DRIFT_CORRECTING,
    LANDING,
    DISARMED
  };

  FlightState flight_state_ = FlightState::INIT;
  uint64_t offboard_setpoint_counter_ = 0;
  int state_timer_count_ = 0;

  // Callback functions
  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void control_loop();

  // Tag tracking state
  bool target_visible_{ false };
  geometry_msgs::msg::PoseStamped::SharedPtr last_target_pose_;
  rclcpp::Time last_target_time_;

  // Helper functions
  void publish_offboard_control_mode();
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
  void publish_position_setpoint(float x, float y, float z);
  void arm();
  void disarm();
};

#endif  // DRIFT_CORRECTION_HPP
