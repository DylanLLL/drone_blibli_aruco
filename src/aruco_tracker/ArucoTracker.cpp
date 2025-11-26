#include "ArucoTracker.hpp"
#include <sstream>

ArucoTrackerNode::ArucoTrackerNode() : Node("aruco_tracker_node")
{
  loadParameters();

  // TODO: params to adjust detector params
  // See: https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html
  _detector_params = std::make_shared<cv::aruco::DetectorParameters>();

  // See: https://docs.opencv.org/4.x/d1/d21/aruco__dictionary_8hpp.html
  _dictionary = std::make_shared<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(_param_dictionary));

  // QoS for image and camera info topics
  auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

  // QoS for pose publisher - match with PrecisionLand subscriber
  auto pose_qos =
	  rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile().history(rclcpp::HistoryPolicy::KeepLast);

  _image_down_sub = create_subscription<sensor_msgs::msg::Image>(
	  "/camera_down/image_raw", sensor_qos, std::bind(&ArucoTrackerNode::image_callback, this, std::placeholders::_1));

  _camera_info_down_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
	  "/camera_down/camera_info", sensor_qos,
	  std::bind(&ArucoTrackerNode::camera_info_callback, this, std::placeholders::_1));

  _image_forward_sub = create_subscription<sensor_msgs::msg::Image>(
	  "/camera_forward/image_raw", sensor_qos,
	  std::bind(&ArucoTrackerNode::image_callback, this, std::placeholders::_1));

  _camera_info_forward_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
	  "/camera_forward/camera_info", sensor_qos,
	  std::bind(&ArucoTrackerNode::camera_info_callback, this, std::placeholders::_1));

  // Publishers
  _image_down_pub = create_publisher<sensor_msgs::msg::Image>("/image_proc_down", sensor_qos);
  _target_pose_down_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose_down", pose_qos);
  _image_forward_pub = create_publisher<sensor_msgs::msg::Image>("/image_proc_forward", sensor_qos);
  _target_pose_forward_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose_forward", pose_qos);
}

void ArucoTrackerNode::loadParameters()
{
  declare_parameter<int>("aruco_id", 0);
  declare_parameter<int>("dictionary", 2);	// DICT_4X4_250
  declare_parameter<double>("marker_size", 0.5);

  get_parameter("aruco_id", _param_aruco_id);
  get_parameter("dictionary", _param_dictionary);
  get_parameter("marker_size", _param_marker_size);
}

void ArucoTrackerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
	// Convert ROS image message to OpenCV image
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	// Detect markers
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners;
	cv::aruco::detectMarkers(cv_ptr->image, _dictionary, corners, ids, _detector_params);

	// If at least one marker is detected
	if (ids.size() > 0)
	{
	  // Draw markers on the image
	  cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

	  // Get the topic name to determine which camera this is from
	  std::string topic = msg->header.frame_id;

	  // Estimate pose of markers
	  std::vector<cv::Vec3d> rvecs, tvecs;
	  cv::aruco::estimatePoseSingleMarkers(corners, _param_marker_size, _camera_matrix, _dist_coeffs, rvecs, tvecs);

	  // Process each detected marker
	  for (size_t i = 0; i < ids.size(); i++)
	  {
		// Draw axis for each marker
		cv::drawFrameAxes(cv_ptr->image, _camera_matrix, _dist_coeffs, rvecs[i], tvecs[i], 0.1);

		// Create pose message
		geometry_msgs::msg::PoseStamped pose_msg;
		pose_msg.header = msg->header;
		pose_msg.header.frame_id = topic;  // Use the camera frame

		// Convert rotation vector to quaternion
		cv::Mat rot_mat;
		cv::Rodrigues(rvecs[i], rot_mat);
		tf2::Matrix3x3 tf_rot_mat(rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2),
								  rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
								  rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2));
		tf2::Quaternion tf_quat;
		tf_rot_mat.getRotation(tf_quat);

		// Fill in pose message
		pose_msg.pose.position.x = tvecs[i][0];
		pose_msg.pose.position.y = tvecs[i][1];
		pose_msg.pose.position.z = tvecs[i][2];
		pose_msg.pose.orientation.x = tf_quat.x();
		pose_msg.pose.orientation.y = tf_quat.y();
		pose_msg.pose.orientation.z = tf_quat.z();
		pose_msg.pose.orientation.w = tf_quat.w();

		// Log detected marker info
		RCLCPP_INFO(get_logger(), "Detected marker ID %d on camera %s at position: x=%.3f, y=%.3f, z=%.3f", ids[i],
					topic.c_str(), tvecs[i][0], tvecs[i][1], tvecs[i][2]);

		// Publish pose based on which camera detected it
		// Check both the original topic and header frame_id since Gazebo might use different naming conventions
		bool is_forward = topic.find("forward") != std::string::npos ||
						  msg->header.frame_id.find("forward") != std::string::npos ||
						  msg->header.frame_id.find("camera_link_forward") != std::string::npos;

		bool is_down = topic.find("down") != std::string::npos ||
					   msg->header.frame_id.find("down") != std::string::npos ||
					   msg->header.frame_id.find("camera_link_down") != std::string::npos;

		if (is_forward)
		{
		  _target_pose_forward_pub->publish(pose_msg);
		  annotate_image(cv_ptr, tvecs[i]);
		  RCLCPP_INFO(get_logger(), "Published forward camera pose for marker ID %d", ids[i]);
		}
		else if (is_down)
		{
		  _target_pose_down_pub->publish(pose_msg);
		  annotate_image(cv_ptr, tvecs[i]);
		  RCLCPP_INFO(get_logger(), "Published down camera pose for marker ID %d", ids[i]);
		}
		else
		{
		  RCLCPP_WARN(get_logger(), "Unrecognized camera: %s / %s", topic.c_str(), msg->header.frame_id.c_str());
		}
	  }
	}

	// Publish processed image
	// Get the topic name to determine which camera this is from
	std::string topic_name = msg->header.frame_id;
	std::string topic_path = msg->header.frame_id;
	RCLCPP_DEBUG(get_logger(), "Processing image from topic: %s", topic_name.c_str());

	// Check both the topic and frame_id for camera identification
	bool is_forward = topic_name.find("forward") != std::string::npos ||
					  topic_path.find("forward") != std::string::npos ||
					  topic_path.find("camera_link_forward") != std::string::npos;

	bool is_down = topic_name.find("down") != std::string::npos || topic_path.find("down") != std::string::npos ||
				   topic_path.find("camera_link_down") != std::string::npos;

	if (is_forward)
	{
	  _image_forward_pub->publish(*cv_ptr->toImageMsg());
	  RCLCPP_DEBUG(get_logger(), "Published processed forward camera image");
	}
	else if (is_down)
	{
	  _image_down_pub->publish(*cv_ptr->toImageMsg());
	  RCLCPP_DEBUG(get_logger(), "Published processed down camera image");
	}
	else
	{
	  RCLCPP_WARN(get_logger(), "Unknown camera frame_id: %s (original path: %s)", topic_name.c_str(),
				  topic_path.c_str());
	}
  }
  catch (const cv_bridge::Exception& e)
  {
	RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void ArucoTrackerNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  // Always update the camera matrix and distortion coefficients from the new message
  _camera_matrix =
	  cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();  // Use clone to ensure a deep copy
  _dist_coeffs =
	  cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();  // Use clone to ensure a deep copy

  // Log the first row of the camera matrix to verify correct values
  RCLCPP_INFO(get_logger(), "Camera matrix updated:\n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
			  _camera_matrix.at<double>(0, 0), _camera_matrix.at<double>(0, 1), _camera_matrix.at<double>(0, 2),
			  _camera_matrix.at<double>(1, 0), _camera_matrix.at<double>(1, 1), _camera_matrix.at<double>(1, 2),
			  _camera_matrix.at<double>(2, 0), _camera_matrix.at<double>(2, 1), _camera_matrix.at<double>(2, 2));
  RCLCPP_INFO(get_logger(), "Camera Matrix: fx=%f, fy=%f, cx=%f, cy=%f", _camera_matrix.at<double>(0, 0),  // fx
			  _camera_matrix.at<double>(1, 1),															   // fy
			  _camera_matrix.at<double>(0, 2),															   // cx
			  _camera_matrix.at<double>(1, 2)															   // cy
  );

  // Check if focal length is zero after update
  if (_camera_matrix.at<double>(0, 0) == 0)
  {
	RCLCPP_ERROR(get_logger(), "Focal length is zero after update!");
  }
  else
  {
	RCLCPP_INFO(get_logger(), "Updated camera intrinsics from camera_info topic.");

	RCLCPP_INFO(get_logger(), "Unsubscribing from camera info topic");
	_camera_info_down_sub.reset();
	_camera_info_forward_sub.reset();
  }
}

void ArucoTrackerNode::annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target)
{
  // Annotate the image with the target position and marker size
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2);
  stream << "X: " << target[0] << " Y: " << target[1] << " Z: " << target[2];
  std::string text_xyz = stream.str();

  int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 1;
  int thickness = 2;
  int baseline = 0;
  cv::Size textSize = cv::getTextSize(text_xyz, fontFace, fontScale, thickness, &baseline);
  baseline += thickness;
  cv::Point textOrg((image->image.cols - textSize.width - 10), (image->image.rows - 10));
  cv::putText(image->image, text_xyz, textOrg, fontFace, fontScale, cv::Scalar(0, 255, 255), thickness, 8);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoTrackerNode>());
  rclcpp::shutdown();
  return 0;
}