// Adapted from multi_lidar_calibration for the MOMA four-lidar pipeline.
// See LICENSE.Apache-2.0 for the original Pixmoving license.

#include <pointcloud_fusion/multi_lidar_calibration_icp.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <pcl/filters/filter.h>

namespace calibration
{
namespace multi_lidar_calibration_icp
{

MultiLidarCalibrationIcp::MultiLidarCalibrationIcp()
: pnh_("~"),
  pointcloud_source_sub_(pnh_, "input/source_pointcloud", 10),
  pointcloud_target_sub_(pnh_, "input/target_pointcloud", 10),
  sync_(SyncPolicy(10), pointcloud_source_sub_, pointcloud_target_sub_)
{
  double sync_slop;
  pnh_.param("sync_slop", sync_slop, 0.05);
  pnh_.param("broadcast_tf", broadcast_tf_, true);
  if (!std::isfinite(sync_slop) || sync_slop <= 0.0) {
    throw std::runtime_error("sync_slop must be positive and finite");
  }
  sync_.setMaxIntervalDuration(ros::Duration(sync_slop));
  // initialize parameters of node
  if (!pnh_.getParam("initial_pose", param_.initial_pose)) {
    param_.initial_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }
  if (param_.initial_pose.size() != 6 ||
      !std::all_of(param_.initial_pose.begin(), param_.initial_pose.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw std::runtime_error("initial_pose must contain six finite XYZ/RPY values");
  }
  pnh_.param("max_iteration", param_.max_iteration, 100);
  pnh_.param("transform_epsilon", param_.transform_epsilon, 1e-9);
  pnh_.param("max_coorespondence_distance", param_.max_coorespondence_distance, 0.05);
  pnh_.param("euclidean_fitness_epsilon", param_.euclidean_fitness_epsilon, 1.0);
  pnh_.param(
    "ransac_outlier_rejection_threshold", param_.ransac_outlier_rejection_threshold, 1.5);

  double leaf_size;
  pnh_.param("leaf_size", leaf_size, 0.2);
  approximate_voxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);

  icp_.setMaximumIterations(param_.max_iteration);
  icp_.setTransformationEpsilon(param_.transform_epsilon);
  icp_.setMaxCorrespondenceDistance(param_.max_coorespondence_distance);
  icp_.setEuclideanFitnessEpsilon(param_.euclidean_fitness_epsilon);
  icp_.setRANSACOutlierRejectionThreshold(param_.ransac_outlier_rejection_threshold);

  Eigen::Translation3f initial_translation(
    param_.initial_pose.at(0), param_.initial_pose.at(1), param_.initial_pose.at(2));

  Eigen::AngleAxisf initial_rotation_x(param_.initial_pose.at(3), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf initial_rotation_y(param_.initial_pose.at(4), Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf initial_rotation_z(param_.initial_pose.at(5), Eigen::Vector3f::UnitZ());
  current_transform_matrix_ =
    (initial_translation * initial_rotation_z * initial_rotation_y * initial_rotation_x)
      .matrix();
  std::cout << "initial guess: " << std::endl << current_transform_matrix_ << std::endl;

  // tf broadcaster
  sync_.registerCallback(std::bind(
    &MultiLidarCalibrationIcp::callbackLidars, this, std::placeholders::_1,
    std::placeholders::_2));
}

MultiLidarCalibrationIcp::~MultiLidarCalibrationIcp()
{
}

void MultiLidarCalibrationIcp::callbackLidars(
  const sensor_msgs::PointCloud2ConstPtr & point_1,
  const sensor_msgs::PointCloud2ConstPtr & point_2)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_pointcloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr final_pointcloud (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source_pointcloud(
    new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_target_pointcloud(
    new pcl::PointCloud<pcl::PointXYZI>);

  if (point_1->width == 0 || point_1->height == 0 ||
      point_2->width == 0 || point_2->height == 0) {
    ROS_WARN_THROTTLE(5.0, "Calibration received an empty cloud");
    return;
  }
  pcl::fromROSMsg(*point_1, *source_pointcloud);
  pcl::fromROSMsg(*point_2, *target_pointcloud);
  std::vector<int> indices;
  source_pointcloud->is_dense = false;
  target_pointcloud->is_dense = false;
  pcl::removeNaNFromPointCloud(*source_pointcloud, *source_pointcloud, indices);
  pcl::removeNaNFromPointCloud(*target_pointcloud, *target_pointcloud, indices);

  approximate_voxel_filter_.setInputCloud(target_pointcloud);
  if (!target_pointcloud->empty()) {
    approximate_voxel_filter_.filter(*filtered_target_pointcloud);
  }
  if (source_pointcloud->size() < 3 || filtered_target_pointcloud->size() < 3) {
    ROS_WARN_THROTTLE(5.0, "Not enough finite points for calibration");
    return;
  }

  icp_.setInputSource(filtered_target_pointcloud);
  icp_.setInputTarget(source_pointcloud);

  icp_.align(*final_pointcloud, current_transform_matrix_);

  if (icp_.hasConverged() && icp_.getFinalTransformation().allFinite())
  {
    current_transform_matrix_ = icp_.getFinalTransformation();
    std::cout << "ICP converged." << std::endl
              << "The score is " << icp_.getFitnessScore() << std::endl;
    std::cout << "Transformation matrix:" << std::endl;
    std::cout << current_transform_matrix_ << std::endl;
    Eigen::Matrix3f rotation_matrix = current_transform_matrix_.block(0, 0, 3, 3);
    Eigen::Vector3f translation_vector = current_transform_matrix_.block(0, 3, 3, 1);
    const Eigen::Quaternionf q(rotation_matrix);
    const tf::Vector3 origin(translation_vector[0], translation_vector[1], translation_vector[2]);
    const tf::Quaternion rotation(q.x(), q.y(), q.z(), q.w());
    double roll, pitch, yaw;
    tf::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
    ROS_INFO("Child-to-parent YAML: {x: %.9g, y: %.9g, z: %.9g, roll: %.9g, pitch: %.9g, yaw: %.9g}",
             origin.x(), origin.y(), origin.z(), roll, pitch, yaw);
    ROS_INFO("Static TF: rosrun tf2_ros static_transform_publisher %.9g %.9g %.9g %.9g %.9g %.9g %.9g %s %s",
             origin.x(), origin.y(), origin.z(), rotation.x(), rotation.y(), rotation.z(), rotation.w(),
             point_1->header.frame_id.c_str(), point_2->header.frame_id.c_str());
    if (broadcast_tf_ && !point_1->header.frame_id.empty() && !point_2->header.frame_id.empty() &&
        point_1->header.frame_id != point_2->header.frame_id) {
      tf_broadcaster_.sendTransform(tf::StampedTransform(
        tf::Transform(rotation, origin), point_1->header.stamp,
        point_1->header.frame_id, point_2->header.frame_id));
    }
  }
}

} // namespace multi_lidar_calibration_icp
} // calibration
