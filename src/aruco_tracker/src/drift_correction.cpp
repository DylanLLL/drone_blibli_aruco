#include "drift_correction_new.hpp"
#include <chrono>
#include <cmath>
#include <px4_msgs/msg/vehicle_command.hpp>

using namespace std::chrono_literals;

DriftCorrection::DriftCorrection() : Node("drift_correction")
{
  // Declare parameters
  this->declare_parameter("desired_distance", 1.5);    // meters
  this->declare_parameter("correction_speed", 0.5);    // m/s
  this->declare_parameter("distance_tolerance", 0.1);  // meters

  desired_distance_ = this->get_parameter("desired_distance").as_double();
  correction_speed_ = this->get_parameter("correction_speed").as_double();
  distance_tolerance_ = this->get_parameter("distance_tolerance").as_double();

  // Validate parameters
  if (correction_speed_ <= 0.0)
  {
    RCLCPP_WARN(this->get_logger(), "Invalid correction_speed: %.2f, setting to default 0.5", correction_speed_);
    correction_speed_ = 0.5;
  }
  if (desired_distance_ <= 0.0)
  {
    RCLCPP_WARN(this->get_logger(), "Invalid desired_distance: %.2f, setting to default 1.5", desired_distance_);
    desired_distance_ = 1.5;
  }
  if (distance_tolerance_ <= 0.0)
  {
    RCLCPP_WARN(this->get_logger(), "Invalid distance_tolerance: %.2f, setting to default 0.1", distance_tolerance_);
    distance_tolerance_ = 0.1;
  }

  // Subscribers
  auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose_forward", qos, std::bind(&DriftCorrection::target_pose_callback, this, std::placeholders::_1));

  // Publishers
  trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
  offboard_control_mode_pub_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
  vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

  // Timer
  timer_ = this->create_wall_timer(100ms, std::bind(&DriftCorrection::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "Drift Correction Node Initialized");
  RCLCPP_INFO(this->get_logger(),
              "Parameters - Desired distance: %.2f m, Correction speed: %.2f m/s, Tolerance: %.2f m", desired_distance_,
              correction_speed_, distance_tolerance_);
  RCLCPP_INFO(this->get_logger(), "Starting flight sequence with offboard control integration...");
}

void DriftCorrection::target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_target_pose_ = msg;
  target_visible_ = true;
  last_target_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "Target pose received: x=%.3f, y=%.3f, z=%.3f", msg->pose.position.x,
              msg->pose.position.y, msg->pose.position.z);
}

void DriftCorrection::publish_offboard_control_mode()
{
  auto offboard_msg = px4_msgs::msg::OffboardControlMode();
  offboard_msg.timestamp = this->now().nanoseconds() / 1000;  // microseconds

  // Enable both position and velocity control
  offboard_msg.position = true;  // Enable position control for hovering
  offboard_msg.velocity = true;  // Enable velocity control for drift correction
  offboard_msg.acceleration = false;
  offboard_msg.attitude = false;
  offboard_msg.body_rate = false;
  offboard_msg.thrust_and_torque = false;
  offboard_msg.direct_actuator = false;

  offboard_control_mode_pub_->publish(offboard_msg);
}

void DriftCorrection::publish_vehicle_command(uint16_t command, float param1, float param2)
{
  auto msg = px4_msgs::msg::VehicleCommand();
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this->now().nanoseconds() / 1000;
  vehicle_command_pub_->publish(msg);
}

void DriftCorrection::publish_position_setpoint(float x, float y, float z)
{
  auto msg = px4_msgs::msg::TrajectorySetpoint();
  msg.position = { x, y, z };
  msg.yaw = 0.0;
  msg.timestamp = this->now().nanoseconds() / 1000;
  trajectory_pub_->publish(msg);
}

void DriftCorrection::arm()
{
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
  RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void DriftCorrection::disarm()
{
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
  RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void DriftCorrection::control_loop()
{
  // Always publish offboard control mode to maintain control
  publish_offboard_control_mode();
  state_timer_count_++;

  // State machine for flight control
  switch (flight_state_)
  {
    case FlightState::INIT:
      // Send initial setpoints before switching to offboard mode
      if (offboard_setpoint_counter_ <= 10)
      {
        offboard_setpoint_counter_++;
        publish_position_setpoint(0.0, 0.0, -3.0);  // Hover at 3m height
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Sending initial setpoints (%lu/10)",
                             offboard_setpoint_counter_);
      }
      else
      {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);  // Set to Offboard
        flight_state_ = FlightState::ARMING;
        state_timer_count_ = 0;
        RCLCPP_INFO(this->get_logger(), "Switching to OFFBOARD mode...");
      }
      break;

    case FlightState::ARMING:
      publish_position_setpoint(0.0, 0.0, -3.0);  // Continue sending setpoints
      if (state_timer_count_ >= 10)               // Wait 1 second (100ms * 10)
      {
        arm();
        flight_state_ = FlightState::OFFBOARD_READY;
        state_timer_count_ = 0;
        RCLCPP_INFO(this->get_logger(), "Arming and preparing for offboard control...");
      }
      break;

    case FlightState::OFFBOARD_READY:
      publish_position_setpoint(0.0, 0.0, -3.0);  // Hold position
      if (state_timer_count_ >= 20)               // Wait 2 seconds for stabilization
      {
        flight_state_ = FlightState::HOVERING;
        state_timer_count_ = 0;
        RCLCPP_INFO(this->get_logger(), "Ready for drift correction - hovering and waiting for target...");
      }
      break;

    case FlightState::HOVERING:
      // Check if we have a target and need drift correction
      if (target_visible_ && last_target_pose_)
      {
        auto time_since_target = this->now() - last_target_time_;
        if (time_since_target.seconds() <= 1.0)  // Target is fresh
        {
          flight_state_ = FlightState::DRIFT_CORRECTING;
          RCLCPP_INFO(this->get_logger(), "Target detected - starting drift correction");
        }
        else
        {
          publish_position_setpoint(0.0, 0.0, -3.0);  // Hold position
        }
      }
      else
      {
        publish_position_setpoint(0.0, 0.0, -3.0);  // Hold position
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Hovering - waiting for target detection...");
      }
      break;

    case FlightState::DRIFT_CORRECTING: {
      // This is where your original drift correction logic goes
      if (!target_visible_ || !last_target_pose_)
      {
        flight_state_ = FlightState::HOVERING;
        RCLCPP_WARN(this->get_logger(), "Lost target - returning to hover mode");
        return;
      }

      // Timeout check for target visibility
      auto time_since_target = this->now() - last_target_time_;
      if (time_since_target.seconds() > 1.0)
      {
        target_visible_ = false;
        flight_state_ = FlightState::HOVERING;
        RCLCPP_WARN(this->get_logger(), "Target timeout - returning to hover mode");
        return;
      }

      // The tag pose is in the camera frame, where Z is forward (distance from tag)
      double current_distance = last_target_pose_->pose.position.z;

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Current distance: %.2f m, Desired: %.2f m, Tolerance: %.2f m", current_distance,
                           desired_distance_, distance_tolerance_);

      // Calculate velocity based on distance error
      double distance_error = current_distance - desired_distance_;
      double vx = 0.0;

      // Only correct if we're outside the tolerance zone
      if (std::abs(distance_error) >= distance_tolerance_)
      {
        // Proportional control - scale velocity by distance error
        vx = std::clamp(distance_error * correction_speed_, -correction_speed_, correction_speed_);
        RCLCPP_INFO(this->get_logger(), "Distance outside tolerance, correcting...");
        RCLCPP_INFO(this->get_logger(), "Distance error: %.2f m, Velocity: %.2f m/s", distance_error, vx);

        // Publish velocity setpoint for drift correction
        auto setpoint = px4_msgs::msg::TrajectorySetpoint();
        setpoint.timestamp = this->now().nanoseconds() / 1000;
        setpoint.position[0] = 1.0;  // X forward/back
        setpoint.position[1] = 0.0;  // Y left/right
        setpoint.position[2] = 0.0;  // Z up/down - maintain altitude
        trajectory_pub_->publish(setpoint);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "Publishing position command: x=%.3f, y=%.3f, z=%.3f", setpoint.position[0],
                             setpoint.position[1], setpoint.position[2]);
      }
      else
      {
        // Within tolerance - hold position
        publish_position_setpoint(0.0, 0.0, -1.5);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Within tolerance, holding position");
      }
      break;
    }

    case FlightState::LANDING:
      publish_position_setpoint(0.0, 0.0, 0.0);  // Land
      if (state_timer_count_ >= 50)              // Wait 5 seconds for landing
      {
        flight_state_ = FlightState::DISARMED;
        disarm();
        RCLCPP_INFO(this->get_logger(), "Landing complete - disarming");
      }
      break;

    case FlightState::DISARMED:
      // Flight complete - do nothing
      break;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriftCorrection>());
  rclcpp::shutdown();
  return 0;
}
