#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <boost/bind/bind.hpp>
#include <dynamic_reconfigure/server.h>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointcloud_fusion/BoundsConfig.h>
#include <pointcloud_fusion/ExtrinsicsConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace
{
using Cloud = sensor_msgs::PointCloud2;
using CloudPtr = sensor_msgs::PointCloud2ConstPtr;
using Points = pcl::PointCloud<pcl::PointXYZI>;
using Bounds = pointcloud_fusion::BoundsConfig;
using Extrinsics = pointcloud_fusion::ExtrinsicsConfig;
using BoundsServer = dynamic_reconfigure::Server<Bounds>;
using ExtrinsicsServer = dynamic_reconfigure::Server<Extrinsics>;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<Cloud, Cloud, Cloud, Cloud>;
using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

bool inside(const pcl::PointXYZI& point, const Bounds& bounds)
{
  return point.x >= bounds.x_min && point.x <= bounds.x_max &&
         point.y >= bounds.y_min && point.y <= bounds.y_max &&
         point.z >= bounds.z_min && point.z <= bounds.z_max;
}

bool validBounds(const Bounds& bounds)
{
  return bounds.x_min <= bounds.x_max && bounds.y_min <= bounds.y_max &&
         bounds.z_min <= bounds.z_max;
}

Eigen::Matrix4f childToParent(const Extrinsics& pose)
{
  const Eigen::AngleAxisf roll(pose.roll, Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf pitch(pose.pitch, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf yaw(pose.yaw, Eigen::Vector3f::UnitZ());
  return (Eigen::Translation3f(pose.x, pose.y, pose.z) * yaw * pitch * roll).matrix();
}

bool finitePoint(const pcl::PointXYZI& point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

class PointCloudFusion
{
public:
  PointCloudFusion() : private_nh_("~")
  {
    private_nh_.param("fusion_lidar_num", lidar_count_, 4);
    private_nh_.param("set_params_tf", transform_points_, true);
    private_nh_.param("extrinsics_ready", extrinsics_ready_, false);
    private_nh_.param("set_params_external_bounds", crop_external_, false);
    private_nh_.param("set_params_internal_bounds", crop_internal_, false);
    private_nh_.param("set_dynamic_params", dynamic_params_, false);
    private_nh_.param<std::string>("topics/fusion_pc_frame_id", expected_parent_frame_, "");

    double sync_slop;
    int queue_size;
    private_nh_.param("sync_slop", sync_slop, 0.05);
    private_nh_.param("sync_queue_size", queue_size, 10);
    if (lidar_count_ < 2 || lidar_count_ > 4 || queue_size < 2 ||
        !std::isfinite(sync_slop) || sync_slop <= 0.0) {
      throw std::runtime_error("Expected 2 to 4 lidar inputs, sync_queue_size >= 2, and sync_slop > 0");
    }

    configureBounds("external_bounds", external_bounds_, external_server_);
    configureBounds("internal_bounds", internal_bounds_, internal_server_);
    for (int i = 0; i < lidar_count_ - 1; ++i) {
      configureExtrinsics(i);
    }
    if (transform_points_ && !extrinsics_ready_) {
      ROS_WARN("Fusion is waiting for validated extrinsics. Set extrinsics_ready:=true after calibration.");
    }

    std::string output;
    private_nh_.param<std::string>("topics/fusion_pc_topic", output, "/pointcloud_fused");
    publisher_ = nh_.advertise<Cloud>(output, 10);
    for (int i = 0; i < lidar_count_; ++i) {
      const std::string parameter = i == 0 ? "topics/parent_pc_topic" :
        "topics/child_pc_topic" + std::to_string(i);
      std::string topic;
      if (!private_nh_.getParam(parameter, topic) || topic.empty()) {
        throw std::runtime_error("Missing point cloud input parameter " + parameter);
      }
      subscribers_[i].subscribe(nh_, topic, queue_size);
    }

    SyncPolicy policy(queue_size);
    policy.setMaxIntervalDuration(ros::Duration(sync_slop));
    // Feed unused synchronizer slots from the parent in two- and three-lidar modes.
    sync_.reset(new Synchronizer(policy, subscribers_[0], subscribers_[1],
      subscribers_[lidar_count_ >= 3 ? 2 : 0], subscribers_[lidar_count_ == 4 ? 3 : 0]));
    using namespace boost::placeholders;
    sync_->registerCallback(boost::bind(&PointCloudFusion::fuse, this, _1, _2, _3, _4));
  }

private:
  void configureExtrinsics(int index)
  {
    const std::string name = "cpc" + std::to_string(index + 1) + "_to_ppc";
    ros::NodeHandle parameters(private_nh_, name);
    Extrinsics pose = Extrinsics::__getDefault__();
    pose.__fromServer__(parameters);
    if (transform_points_ && extrinsics_ready_) {
      for (const char* field : {"x", "y", "z", "roll", "pitch", "yaw"}) {
        double value;
        if (!parameters.getParam(field, value) || !std::isfinite(value)) {
          throw std::runtime_error("Missing or invalid extrinsic " + name + "/" + field);
        }
      }
    }
    transforms_[index] = childToParent(pose);
    if (!dynamic_params_) {
      return;
    }
    // Server construction clamps the loaded parameters before invoking callbacks.
    // Keep the calibrated transform if the original tuning sliders cannot represent it.
    Extrinsics clamped = pose;
    clamped.__clamp__();
    if (clamped.x != pose.x || clamped.y != pose.y || clamped.z != pose.z ||
        clamped.roll != pose.roll || clamped.pitch != pose.pitch || clamped.yaw != pose.yaw) {
      ROS_WARN("Online tuning disabled for %s: YAML extrinsics are outside the slider limits", name.c_str());
      return;
    }
    auto& server = extrinsics_servers_[index];
    server.reset(new ExtrinsicsServer(parameters));
    server->setCallback([this, index](Extrinsics& value, uint32_t) {
      transforms_[index] = childToParent(value);
    });
  }

  void configureBounds(const std::string& name, Bounds& bounds,
                       std::unique_ptr<BoundsServer>& server)
  {
    ros::NodeHandle parameters(private_nh_, name);
    bounds = Bounds::__getDefault__();
    bounds.__fromServer__(parameters);
    if (!validBounds(bounds)) {
      throw std::runtime_error("Invalid bounds in " + name);
    }
    if (!dynamic_params_) {
      return;
    }
    Bounds clamped = bounds;
    clamped.__clamp__();
    if (clamped.x_min != bounds.x_min || clamped.x_max != bounds.x_max ||
        clamped.y_min != bounds.y_min || clamped.y_max != bounds.y_max ||
        clamped.z_min != bounds.z_min || clamped.z_max != bounds.z_max) {
      ROS_WARN("Online tuning disabled for %s: YAML bounds are outside the slider limits", name.c_str());
      return;
    }
    server.reset(new BoundsServer(parameters));
    Bounds* const current = &bounds;
    server->setCallback([current](Bounds& value, uint32_t) {
      if (validBounds(value)) {
        *current = value;
      } else {
        ROS_WARN("Rejected crop box with min greater than max");
        value = *current;
      }
    });
  }

  void fuse(const CloudPtr& parent, const CloudPtr& first, const CloudPtr& second, const CloudPtr& third)
  {
    const std::array<CloudPtr, 4> messages{{parent, first, second, third}};
    if (transform_points_ && !extrinsics_ready_) {
      return;
    }
    const auto& header = messages[0]->header;
    if (header.frame_id.empty() ||
        (!expected_parent_frame_.empty() && header.frame_id != expected_parent_frame_)) {
      ROS_WARN_THROTTLE(5.0, "Unexpected parent cloud frame: '%s'", header.frame_id.c_str());
      return;
    }
    Points fused;
    for (int i = 0; i < lidar_count_; ++i) {
      if (messages[i]->width == 0 || messages[i]->height == 0) {
        continue;
      }
      Points cloud;
      pcl::fromROSMsg(*messages[i], cloud);
      if (i > 0 && transform_points_) {
        pcl::transformPointCloud(cloud, cloud, transforms_[i - 1]);
      }
      for (const auto& point : cloud) {
        if (finitePoint(point) &&
            (!crop_external_ || inside(point, external_bounds_)) &&
            (!crop_internal_ || !inside(point, internal_bounds_))) {
          fused.push_back(point);
        }
      }
    }
    Cloud output;
    pcl::toROSMsg(fused, output);
    output.header = header;
    publisher_.publish(output);
  }

  ros::NodeHandle nh_, private_nh_;
  std::array<message_filters::Subscriber<Cloud>, 4> subscribers_;
  ros::Publisher publisher_;
  std::unique_ptr<Synchronizer> sync_;
  std::string expected_parent_frame_;
  int lidar_count_;
  bool transform_points_, extrinsics_ready_, crop_external_, crop_internal_, dynamic_params_;
  std::array<Eigen::Matrix4f, 3> transforms_;
  Bounds external_bounds_, internal_bounds_;
  std::array<std::unique_ptr<ExtrinsicsServer>, 3> extrinsics_servers_;
  std::unique_ptr<BoundsServer> external_server_, internal_server_;
};
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_fusion");
  try {
    PointCloudFusion fusion;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("%s", error.what());
    return 1;
  }
}
