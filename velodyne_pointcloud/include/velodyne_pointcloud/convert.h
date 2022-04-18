/* -*- mode: C++ -*- */
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

#ifndef VELODYNE_POINTCLOUD__CONVERT_H_
#define VELODYNE_POINTCLOUD__CONVERT_H_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <velodyne_pointcloud/pointcloudXYZIRADT.h>
#include <velodyne_pointcloud/rawdata.h>

#include <deque>
#include <string>

namespace velodyne_pointcloud
{
class Convert : public rclcpp::Node
{
public:
  Convert(const rclcpp::NodeOptions & options);
  ~Convert() {}

private:
  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);
  void processScan(const velodyne_msgs::msg::VelodyneScan::SharedPtr scanMsg);
  visualization_msgs::msg::MarkerArray createVelodyneModelMakerMsg(
    const std_msgs::msg::Header & header);
  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    tf2::Transform * tf2_transform_ptr);

  rclcpp::Subscription<velodyne_msgs::msg::VelodyneScan>::SharedPtr velodyne_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_ex_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_invalid_near_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_combined_ex_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

  // tf2_ros::Buffer tf2_buffer_;
  // tf2_ros::TransformListener tf2_listener_;

  // Buffer for overflow points
  velodyne_pointcloud::PointcloudXYZIRADT _overflow_buffer;
  /// Pointer to dynamic reconfigure service srv_
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  std::shared_ptr<velodyne_rawdata::RawData> data_;

  int num_points_threshold_;
  std::vector<float> invalid_intensity_array_;
  std::string base_link_frame_;

  /// configuration parameters
  typedef struct
  {
    double min_range;
    double max_range;
    double view_direction;
    double view_width;
    int npackets;           ///< number of packets to combine
    double scan_phase;      ///< sensor phase (degrees)
    bool sensor_timestamp;  ///< flag on whether to use sensor (GPS) time or ROS receive time
  } Config;
  Config config_;
};

}  // namespace velodyne_pointcloud

#endif  // VELODYNE_POINTCLOUD__CONVERT_H_
