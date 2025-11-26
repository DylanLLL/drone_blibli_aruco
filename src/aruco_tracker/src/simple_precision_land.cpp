#include "simple_precision_land.hpp"
#include <chrono>

using namespace std::chrono_literals;

SimplePrecisionLand::SimplePrecisionLand() : Node("simple_precision_land")
{
  // Declare and get parameters
  this->declare_parameter("descent_velocity", -0.5);  // m/s
  this->declare_parameter("hover_height", 3.0);       // meters
  this->declare_parameter("land_threshold", 0.1);     // meters

  descent_velocity_ = this->get_parameter("descent_velocity").as_double();
  hover_height_ = this->get_parameter("hover_height").as_double();
  land_threshold_ = this->get_parameter("land_threshold").as_double();

  // Create subscribers
  auto qos = rclcpp::QoS(1).best_effort().durability_volatile();

  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose_down", qos, std::bind(&SimplePrecisionLand::target_pose_callback, this, std::placeholders::_1));
  // /target_pose_down

  land_detected_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", qos,
      std::bind(&SimplePrecisionLand::land_detected_callback, this, std::placeholders::_1));

  // Create publishers
  trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

  vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

  // Create timer for control loop
  timer_ = this->create_wall_timer(100ms, std::bind(&SimplePrecisionLand::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "Simple Precision Land Node Initialized");
}

void SimplePrecisionLand::target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_target_pose_ = msg;
  target_visible_ = true;
  last_target_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "Target pose received: x=%.3f, y=%.3f, z=%.3f", msg->pose.position.x,
              msg->pose.position.y, msg->pose.position.z);
}

void SimplePrecisionLand::land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
  has_landed_ = msg->landed;
  if (has_landed_)
  {
    RCLCPP_INFO(this->get_logger(), "Landing detected!");
  }
}

void SimplePrecisionLand::publish_land_command()
{
  auto command_msg = px4_msgs::msg::VehicleCommand();
  command_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
  command_msg.param5 = NAN;  // Latitude
  command_msg.param6 = NAN;  // Longitude
  command_msg.param7 = NAN;  // Altitude
  command_msg.target_system = 1;
  command_msg.target_component = 1;
  command_msg.source_system = 1;
  command_msg.source_component = 1;
  command_msg.from_external = true;
  command_msg.timestamp = this->now().nanoseconds() / 1000;  // microseconds

  vehicle_command_pub_->publish(command_msg);
  RCLCPP_INFO(this->get_logger(), "Published land command");
}

void SimplePrecisionLand::control_loop()
{
  // Check if target is still visible (timeout after 1 second)
  if (target_visible_ && last_target_pose_)
  {
    auto time_since_target = this->now() - last_target_time_;
    if (time_since_target.seconds() > 1.0)
    {
      target_visible_ = false;
      RCLCPP_WARN(this->get_logger(), "Target timeout - no target seen for %.2f seconds", time_since_target.seconds());
    }
  }

  if (!target_visible_ || !last_target_pose_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No target visible (visible: %s, pose: %s)",
                         target_visible_ ? "true" : "false", last_target_pose_ ? "valid" : "null");
    return;
  }

  if (has_landed_)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Vehicle has landed");
    return;
  }

  auto setpoint = px4_msgs::msg::TrajectorySetpoint();
  setpoint.timestamp = this->now().nanoseconds() / 1000;  // microseconds

  // Position setpoint from ArUco tag with fixed height
  setpoint.position[0] = last_target_pose_->pose.position.x;  // x
  setpoint.position[1] = last_target_pose_->pose.position.y;  // y

  if (!is_landing_)
  {
    // Hover above target
    setpoint.position[2] = -hover_height_;  // z (NED frame, negative is up)

    // Check if we're close enough to start landing
    double xy_error = std::hypot(last_target_pose_->pose.position.x, last_target_pose_->pose.position.y);
    if (xy_error < land_threshold_)
    {
      RCLCPP_INFO(this->get_logger(), "Starting descent");
      is_landing_ = true;
      publish_land_command();
    }
  }
  else
  {
    // Descend with constant velocity
    setpoint.velocity[2] = -descent_velocity_;  // Negative velocity for descent in NED frame
  }

  trajectory_pub_->publish(setpoint);
}

// void SimplePrecisionLand::control_loop()
// {
//   if (!target_visible_ || !last_target_pose_ || has_landed_)
//     return;

//   auto setpoint = px4_msgs::msg::TrajectorySetpoint();
//   setpoint.timestamp = this->now().nanoseconds() / 1000;

//   // Maintain altitude
//   setpoint.position[2] = -hover_height_;

//   // Forward/backward distance to target
//   double distance_error = last_target_pose_->pose.position.z - 1.5;  // want 1.5m away
//   double vx = 0.0;

//   // Only move if outside tolerance
//   if (std::abs(distance_error) > 0.1)
//   {
//     vx = (distance_error > 0) ? 0.5 : -0.5;  // m/s forward/back
//     RCLCPP_INFO(this->get_logger(), "Approaching target, distance_error=%.2f, vx=%.2f", distance_error, vx);
//   }
//   else
//   {
//     vx = 0.0;
//     RCLCPP_INFO(this->get_logger(), "Reached 1.5m in front of target, holding position");
//   }

//   // Apply body-frame forward velocity
//   setpoint.velocity[0] = vx;
//   setpoint.velocity[1] = 0.0;  // lateral
//   setpoint.velocity[2] = 0.0;  // keep altitude

//   trajectory_pub_->publish(setpoint);
// }

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePrecisionLand>());
  rclcpp::shutdown();
  return 0;
}
