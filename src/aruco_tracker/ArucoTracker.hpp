#pragma once
#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class ArucoTrackerNode : public rclcpp::Node
{
public:
  // Constructor
  ArucoTrackerNode();

private:
  void loadParameters();

  // Callbacks
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target);

  // ROS2 Subscribers and Publishers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_down_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_down_sub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_down_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_down_pub;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_forward_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_forward_sub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_forward_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_forward_pub;

  // Data
  cv::Ptr<cv::aruco::Dictionary> _dictionary;
  cv::Ptr<cv::aruco::DetectorParameters> _detector_params;
  cv::Mat _camera_matrix;
  cv::Mat _dist_coeffs;

  // State
  std::array<double, 3> _target;
  int _param_aruco_id{};
  int _param_dictionary{};
  double _param_marker_size{};
};
