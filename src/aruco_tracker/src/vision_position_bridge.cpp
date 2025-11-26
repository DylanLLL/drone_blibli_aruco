#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class VisionPositionBridge : public rclcpp::Node
{
public:
  VisionPositionBridge() : Node("vision_position_bridge")
  {
    // Subscribe to ArUco pose
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose_down", 10, std::bind(&VisionPositionBridge::pose_callback, this, std::placeholders::_1));

    // Publish vision odometry to PX4
    vision_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

    RCLCPP_INFO(this->get_logger(), "Vision Position Bridge started");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: /target_pose_down");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /fmu/in/vehicle_visual_odometry");
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    px4_msgs::msg::VehicleOdometry odom_msg{};

    // Convert camera frame to NED (North-East-Down) frame used by PX4
    // Camera frame (OpenCV): X right, Y down, Z forward
    // NED frame (PX4): X forward (North), Y right (East), Z down
    // Body frame: X forward, Y right, Z down

    // For a downward-facing camera:
    // Camera X (right) -> Body Y (right)
    // Camera Y (down) -> Body Z (down)
    // Camera Z (forward) -> Body X (forward)

    // But we need to invert because the marker pose is relative to camera
    // The drone is above the marker, so we need to invert the transformation
    float x_camera = msg->pose.position.x;
    float y_camera = msg->pose.position.y;
    float z_camera = msg->pose.position.z;

    // Transform: Camera sees marker, but we want drone position relative to marker
    // Invert the transformation
    float x_ned = -x_camera;  // Invert X
    float y_ned = -y_camera;  // Invert Y
    float z_ned = -z_camera;  // Invert Z (altitude)

    odom_msg.position[0] = x_ned;
    odom_msg.position[1] = y_ned;
    odom_msg.position[2] = z_ned;

    // Convert quaternion from camera frame to NED frame
    // For now, we'll use a simplified approach - just pass through the orientation
    odom_msg.q[0] = msg->pose.orientation.w;
    odom_msg.q[1] = msg->pose.orientation.x;
    odom_msg.q[2] = msg->pose.orientation.y;
    odom_msg.q[3] = msg->pose.orientation.z;

    // Set velocity to NAN (we don't have velocity from ArUco)
    odom_msg.velocity[0] = NAN;
    odom_msg.velocity[1] = NAN;
    odom_msg.velocity[2] = NAN;

    // Set angular velocity to NAN
    odom_msg.angular_velocity[0] = NAN;
    odom_msg.angular_velocity[1] = NAN;
    odom_msg.angular_velocity[2] = NAN;

    // Position variance (tune based on your system)
    // Lower = more trust in vision
    // Higher = less trust in vision
    odom_msg.position_variance[0] = 0.01f;  // 1cm standard deviation
    odom_msg.position_variance[1] = 0.01f;
    odom_msg.position_variance[2] = 0.01f;

    // Orientation variance
    odom_msg.orientation_variance[0] = 0.05f;
    odom_msg.orientation_variance[1] = 0.05f;
    odom_msg.orientation_variance[2] = 0.05f;

    // Set frames
    odom_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    odom_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

    // Timestamp (PX4 uses microseconds)
    odom_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    odom_msg.timestamp_sample = odom_msg.timestamp;

    // Publish
    vision_odom_pub_->publish(odom_msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Vision position: [%.3f, %.3f, %.3f] m", x_ned,
                         y_ned, z_ned);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vision_odom_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionPositionBridge>());
  rclcpp::shutdown();
  return 0;
}
