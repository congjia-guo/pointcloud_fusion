// Adapted from multi_lidar_calibration for the MOMA four-lidar pipeline.
// See LICENSE.Apache-2.0 for the original Pixmoving license.

#include <exception>
#include <pointcloud_fusion/multi_lidar_calibration_ndt.hpp>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "multi_lidar_calibration_ndt");
  try {
    calibration::multi_lidar_calibration_ndt::MultiLidarCalibrationNdt node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("%s", error.what());
    return 1;
  }
  return 0;
}
