/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include <velodyne_pointcloud/convert.h>

#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/pointcloudXYZIRADT.h>

#include <yaml-cpp/yaml.h>

#include <velodyne_pointcloud/func.h>

namespace velodyne_pointcloud
{

/** \brief For parameter service callback */
template <typename T>
bool get_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
    return parameter.get_name() == name;
  });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}

inline std::chrono::nanoseconds toChronoNanoSeconds(const double seconds)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(seconds));
}

/** @brief Constructor. */
Convert::Convert(const rclcpp::NodeOptions & options)
: Node("velodyne_convert_node", options),
  // tf2_listener_(tf2_buffer_),
  num_points_threshold_(300),
  base_link_frame_("base_link")
{
  data_ = std::make_shared<velodyne_rawdata::RawData>(this);

  RCLCPP_INFO(this->get_logger(), "This node is only tested for VLP16, VLP32C, and VLS128. Use other models at your own risk.");

  // get path to angles.config file for this device
  std::string calibration_file = this->declare_parameter("calibration", "");

  rcl_interfaces::msg::ParameterDescriptor min_range_desc;
  min_range_desc.name = "min_range";
  min_range_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  min_range_desc.description = "minimum range to publish";
  rcl_interfaces::msg::FloatingPointRange min_range_range;
  min_range_range.from_value = 0.1;
  min_range_range.to_value = 10.0;
  min_range_desc.floating_point_range.push_back(min_range_range);
  config_.min_range = this->declare_parameter("min_range", 0.9, min_range_desc);

  rcl_interfaces::msg::ParameterDescriptor max_range_desc;
  max_range_desc.name = "max_range";
  max_range_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  max_range_desc.description = "maximum range to publish";
  rcl_interfaces::msg::FloatingPointRange max_range_range;
  max_range_range.from_value = 0.1;
  max_range_range.to_value = 250.0;
  max_range_desc.floating_point_range.push_back(max_range_range);
  config_.max_range = this->declare_parameter("max_range", 130.0, max_range_desc);

  rcl_interfaces::msg::ParameterDescriptor view_direction_desc;
  view_direction_desc.name = "view_direction";
  view_direction_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  view_direction_desc.description = "angle defining the center of view";
  rcl_interfaces::msg::FloatingPointRange view_direction_range;
  view_direction_range.from_value = -M_PI;
  view_direction_range.to_value = M_PI;
  view_direction_desc.floating_point_range.push_back(view_direction_range);
  config_.view_direction = this->declare_parameter("view_direction", 0.0, view_direction_desc);

  rcl_interfaces::msg::ParameterDescriptor view_width_desc;
  view_width_desc.name = "view_width";
  view_width_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  view_width_desc.description = "angle defining the view width";
  rcl_interfaces::msg::FloatingPointRange view_width_range;
  view_width_range.from_value = 0.0;
  view_width_range.to_value = 2.0 * M_PI;
  view_width_desc.floating_point_range.push_back(view_width_range);
  config_.view_width = this->declare_parameter("view_width", 2.0 * M_PI, view_width_desc);

  rcl_interfaces::msg::ParameterDescriptor num_points_threshold_desc;
  num_points_threshold_desc.name = "num_points_threshold";
  num_points_threshold_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  num_points_threshold_desc.description = "num_points_threshold";
  rcl_interfaces::msg::IntegerRange num_points_threshold_range;
  num_points_threshold_range.from_value = 1;
  num_points_threshold_range.to_value = 10000;
  num_points_threshold_desc.integer_range.push_back(num_points_threshold_range);
  num_points_threshold_ = this->declare_parameter("num_points_threshold", 300, num_points_threshold_desc);

  rcl_interfaces::msg::ParameterDescriptor scan_phase_desc;
  scan_phase_desc.name = "scan_phase";
  scan_phase_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  scan_phase_desc.description = "start/end phase for the scan (in degrees)";
  rcl_interfaces::msg::FloatingPointRange scan_phase_range;
  scan_phase_range.from_value = 0.0;
  scan_phase_range.to_value = 359.0;
  scan_phase_desc.floating_point_range.push_back(scan_phase_range);
  config_.scan_phase = this->declare_parameter("scan_phase", 0.0, scan_phase_desc);

  RCLCPP_INFO(this->get_logger(), "correction angles: %s", calibration_file.c_str());

  data_->setup();
  data_->setParameters(
    config_.min_range, config_.max_range, config_.view_direction, config_.view_width);

  std::vector<double> invalid_intensity_double;
  invalid_intensity_double = this->declare_parameter<std::vector<double>>("invalid_intensity");
  // YAML::Node invalid_intensity_yaml = YAML::Load(invalid_intensity);
  invalid_intensity_array_ = std::vector<float>(data_->getNumLasers(), 0);
  for (size_t i = 0; i < invalid_intensity_double.size(); ++i) {
    invalid_intensity_array_.at(i) = static_cast<float>(invalid_intensity_double[i]);
  }

  // advertise
  velodyne_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points", rclcpp::SensorDataQoS());
  velodyne_points_ex_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points_ex", rclcpp::SensorDataQoS());
  velodyne_points_invalid_near_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points_invalid_near", rclcpp::SensorDataQoS());
  velodyne_points_combined_ex_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points_combined_ex", rclcpp::SensorDataQoS());
  marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("velodyne_model_marker", 1);
  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&Convert::paramCallback, this, _1));


  // subscribe to VelodyneScan packets
  velodyne_scan_ =
    this->create_subscription<velodyne_msgs::msg::VelodyneScan>(
    "velodyne_packets", rclcpp::SensorDataQoS(),
    std::bind(&Convert::processScan, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult Convert::paramCallback(const std::vector<rclcpp::Parameter> & p)
{
  RCLCPP_INFO(this->get_logger(), "Reconfigure Request");

  if(get_param(p, "min_range", config_.min_range) ||
     get_param(p, "max_range", config_.max_range) ||
     get_param(p, "view_direction", config_.view_direction) ||
     get_param(p, "view_width", config_.view_width))
  {
    data_->setParameters(
      config_.min_range, config_.max_range, config_.view_direction, config_.view_width);
  }

  get_param(p, "num_points_threshold", num_points_threshold_);
  get_param(p, "scan_phase", config_.scan_phase);

  std::vector<double> invalid_intensity_double;
  auto it = std::find_if(p.cbegin(), p.cend(), [](const rclcpp::Parameter & parameter) {
    return parameter.get_name() == "invalid_intensity";
  });
  if (it != p.cend()) {
    invalid_intensity_double = it->as_double_array();
  }

  invalid_intensity_array_ = std::vector<float>(data_->getNumLasers(), 0);
  for (size_t i = 0; i < invalid_intensity_double.size(); ++i) {
    invalid_intensity_array_.at(i) = static_cast<float>(invalid_intensity_double[i]);
  }

  // if(get_param(p, "invalid_intensity", invalid_intensity))
  // {
    // YAML::Node invalid_intensity_yaml = YAML::Load(invalid_intensity);
    // invalid_intensity_array_ = std::vector<float>(data_->getNumLasers(), 0);
    // for (size_t i = 0; i < invalid_intensity_yaml.size(); ++i) {
    //   invalid_intensity_array_.at(i) = invalid_intensity_yaml[i].as<float>();
    // }
  // }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const velodyne_msgs::msg::VelodyneScan::SharedPtr scanMsg)
{
  velodyne_pointcloud::PointcloudXYZIRADT scan_points_xyziradt;
  if (
    velodyne_points_pub_->get_subscription_count() > 0 ||
    velodyne_points_ex_pub_->get_subscription_count() > 0 ||
    velodyne_points_invalid_near_pub_->get_subscription_count() > 0 ||
    velodyne_points_combined_ex_pub_->get_subscription_count() > 0) {
    scan_points_xyziradt.pc->points.reserve(scanMsg->packets.size() * data_->scansPerPacket() + _overflow_buffer.pc->points.size());

    // Add the overflow buffer points
    for (size_t i = _overflow_buffer.pc->points.size(); i > 0; --i) {
      scan_points_xyziradt.pc->points.push_back(_overflow_buffer.pc->points[i-1]);
    }
    // Reset overflow buffer
    _overflow_buffer.pc->points.clear();
    _overflow_buffer.pc->width = 0;
    _overflow_buffer.pc->height = 1;

    for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
      data_->unpack(scanMsg->packets[i], scan_points_xyziradt);
    }
    // Remove overflow points and add to overflow buffer for next scan
    int phase = (uint16_t)round(config_.scan_phase*100);
    if (scan_points_xyziradt.pc->points.size() > 0)
    {
      uint16_t current_azimuth = (int)scan_points_xyziradt.pc->points.back().azimuth;
      uint16_t phase_diff = (36000 + current_azimuth - phase) % 36000;
      while (phase_diff < 18000 && scan_points_xyziradt.pc->points.size() > 0)
      {
        _overflow_buffer.pc->points.push_back(scan_points_xyziradt.pc->points.back());
        scan_points_xyziradt.pc->points.pop_back();
        current_azimuth = (int)scan_points_xyziradt.pc->points.back().azimuth;
        phase_diff = (36000 + current_azimuth - phase) % 36000;
      }
      _overflow_buffer.pc->width = _overflow_buffer.pc->points.size();
    }

    scan_points_xyziradt.pc->header = pcl_conversions::toPCL(scanMsg->header);

    // Find timestamp from first/last point (and maybe average)?
    double first_point_timestamp = scan_points_xyziradt.pc->points.front().time_stamp;
    // double last_point_timestamp = scan_points_xyziradt.pc->points.back().time_stamp;
    // double average_timestamp = (first_point_timestamp + last_point_timestamp)/2;
    scan_points_xyziradt.pc->header.stamp =
      pcl_conversions::toPCL(rclcpp::Time(toChronoNanoSeconds(first_point_timestamp).count()));
      //pcl_conversions::toPCL(scanMsg->packets[0].stamp - ros::Duration(0.0));
    scan_points_xyziradt.pc->height = 1;
    scan_points_xyziradt.pc->width = scan_points_xyziradt.pc->points.size();
  }

  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr valid_points_xyziradt(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  if (
    velodyne_points_pub_->get_subscription_count() > 0 ||
    velodyne_points_ex_pub_->get_subscription_count() > 0 ||
    velodyne_points_combined_ex_pub_->get_subscription_count() > 0) {
    valid_points_xyziradt =
      extractValidPoints(scan_points_xyziradt.pc, data_->getMinRange(), data_->getMaxRange());
    if (velodyne_points_pub_->get_subscription_count() > 0) {
      const auto valid_points_xyzir = convert(valid_points_xyziradt);
      auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(*valid_points_xyzir, *ros_pc_msg_ptr);
      velodyne_points_pub_->publish(std::move(ros_pc_msg_ptr));
    }
    if (velodyne_points_ex_pub_->get_subscription_count() > 0) {
      auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(*valid_points_xyziradt, *ros_pc_msg_ptr);
      velodyne_points_ex_pub_->publish(std::move(ros_pc_msg_ptr));
    }
  }

  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr invalid_near_points_filtered_xyziradt(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  if (
    velodyne_points_invalid_near_pub_->get_subscription_count() > 0 ||
    velodyne_points_combined_ex_pub_->get_subscription_count() > 0) {
    const size_t num_lasers = data_->getNumLasers();
    const auto sorted_invalid_points_xyziradt = sortZeroIndex(scan_points_xyziradt.pc, num_lasers);
    invalid_near_points_filtered_xyziradt = extractInvalidNearPointsFiltered(
      sorted_invalid_points_xyziradt, invalid_intensity_array_, num_lasers, num_points_threshold_);
    if (velodyne_points_invalid_near_pub_->get_subscription_count() > 0) {
      const auto invalid_near_points_filtered_xyzir =
        convert(invalid_near_points_filtered_xyziradt);
      auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(*invalid_near_points_filtered_xyzir, *ros_pc_msg_ptr);
      velodyne_points_invalid_near_pub_->publish(std::move(ros_pc_msg_ptr));
    }
  }

  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr combined_points_xyziradt(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  if (velodyne_points_combined_ex_pub_->get_subscription_count() > 0) {
    combined_points_xyziradt->points.reserve(
      valid_points_xyziradt->points.size() + invalid_near_points_filtered_xyziradt->points.size());
    combined_points_xyziradt->points.insert(
      std::end(combined_points_xyziradt->points), std::begin(valid_points_xyziradt->points),
      std::end(valid_points_xyziradt->points));
    combined_points_xyziradt->points.insert(
      std::end(combined_points_xyziradt->points),
      std::begin(invalid_near_points_filtered_xyziradt->points),
      std::end(invalid_near_points_filtered_xyziradt->points));
    combined_points_xyziradt->header = valid_points_xyziradt->header;
    combined_points_xyziradt->height = 1;
    combined_points_xyziradt->width = combined_points_xyziradt->points.size();
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*combined_points_xyziradt, *ros_pc_msg_ptr);
    velodyne_points_combined_ex_pub_->publish(std::move(ros_pc_msg_ptr));
  }

  if (marker_array_pub_->get_subscription_count() > 0) {
    const auto velodyne_model_marker = createVelodyneModelMakerMsg(scanMsg->header);
    marker_array_pub_->publish(velodyne_model_marker);
  }
}

visualization_msgs::msg::MarkerArray Convert::createVelodyneModelMakerMsg(
  const std_msgs::msg::Header & header)
{
  auto generatePoint = [](double x, double y, double z) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  };

  auto generateQuaternion = [](double roll, double pitch, double yaw) {
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(roll, pitch, yaw);
    return tf2::toMsg(tf_quat);;
  };

  auto generateVector3 = [](double x, double y, double z) {
    geometry_msgs::msg::Vector3 vec;
    vec.x = x;
    vec.y = y;
    vec.z = z;
    return vec;
  };

  auto generateColor = [](float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  };

  //array[0]:bottom body, array[1]:middle body(laser window), array[2]: top body, array[3]:cable
  const double radius = 0.1033;
  const std::array<geometry_msgs::msg::Point, 4> pos = {
    generatePoint(0.0, 0.0, -0.0285), generatePoint(0.0, 0.0, 0.0), generatePoint(0.0, 0.0, 0.0255),
    generatePoint(-radius / 2.0 - 0.005, 0.0, -0.03)};
  const std::array<geometry_msgs::msg::Quaternion, 4> quta = {
    generateQuaternion(0.0, 0.0, 0.0), generateQuaternion(0.0, 0.0, 0.0),
    generateQuaternion(0.0, 0.0, 0.0), generateQuaternion(0.0, M_PI_2, 0.0)};
  const std::array<geometry_msgs::msg::Vector3, 4> scale = {
    generateVector3(radius, radius, 0.020), generateVector3(radius, radius, 0.037),
    generateVector3(radius, radius, 0.015), generateVector3(0.0127, 0.0127, 0.02)};
  const std::array<std_msgs::msg::ColorRGBA, 4> color = {
    generateColor(0.85, 0.85, 0.85, 0.85), generateColor(0.1, 0.1, 0.1, 0.98),
    generateColor(0.85, 0.85, 0.85, 0.85), generateColor(0.2, 0.2, 0.2, 0.98)};

  visualization_msgs::msg::MarkerArray marker_array_msg;
  for (size_t i = 0; i < 4; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = std::string(header.frame_id) + "_velodyne_model";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = pos[i];
    marker.pose.orientation = quta[i];
    marker.scale = scale[i];
    marker.color = color[i];
    marker_array_msg.markers.push_back(marker);
  }

  return marker_array_msg;
}

// looks like this function is never used
// bool Convert::getTransform(
//   const std::string & target_frame, const std::string & source_frame,
//   tf2::Transform * tf2_transform_ptr)
// {
//   if (target_frame == source_frame) {
//     tf2_transform_ptr->setOrigin(tf2::Vector3(0, 0, 0));
//     tf2_transform_ptr->setRotation(tf2::Quaternion(0, 0, 0, 1));
//     return true;
//   }

//   try {
//     const auto transform_msg =
//       tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
//     tf2::convert(transform_msg.transform, *tf2_transform_ptr);
//   } catch (tf2::TransformException & ex) {
//     RCLCPP_WARN(this->get_logger(), "%s", ex.what());
//     RCLCPP_ERROR(this->get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

//     tf2_transform_ptr->setOrigin(tf2::Vector3(0, 0, 0));
//     tf2_transform_ptr->setRotation(tf2::Quaternion(0, 0, 0, 1));
//     return false;
//   }
//   return true;
// }

}  // namespace velodyne_pointcloud

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(velodyne_pointcloud::Convert)
