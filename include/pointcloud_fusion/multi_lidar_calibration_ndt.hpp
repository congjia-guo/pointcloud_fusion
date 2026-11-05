// Adapted from multi_lidar_calibration for the MOMA four-lidar pipeline.
// See LICENSE.Apache-2.0 for the original Pixmoving license.

#ifndef __MULTI_LIDAR_CALIBRATION_NDT__HPP__
#define __MULTI_LIDAR_CALIBRATION_NDT__HPP__

#include <string>
#include <memory>
#include <vector>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>


namespace calibration
{
namespace multi_lidar_calibration_ndt
{

struct Param
{
  // std::string source_pointcloud_topic; // pointlcoud source topic
  // std::string target_pointcloud_topic; // pointcloud target topic
  std::vector<double> initial_pose; // x y z r p y
  // downsampling parameters
  double leaf_size;
  // ndt parameters
  int max_iteration;
  double transform_epsilon;
  double step_size;
  double resolution;
};

class MultiLidarCalibrationNdt
{
private:
  ros::NodeHandle pnh_;
  Param param_;
  bool broadcast_tf_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_source_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_target_sub_;

  // downsampling filter
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter_;

  // ndt registration
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;

  // transform matrix
  Eigen::Matrix4f current_transform_matrix_;

  // tf
  tf::TransformBroadcaster tf_broadcaster_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> SyncExact;
  SyncExact sync_;

public:
  MultiLidarCalibrationNdt();
  ~MultiLidarCalibrationNdt();
  void callbackLidars(
    const sensor_msgs::PointCloud2ConstPtr & point_1,
    const sensor_msgs::PointCloud2ConstPtr & point_2);
};


}
}

#endif // __MULTI_LIDAR_CALIBRATION_NDT__HPP__
