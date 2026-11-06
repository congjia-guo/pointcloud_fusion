#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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
#include <pointcloud_fusion/FilterConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace
{
using Cloud = sensor_msgs::PointCloud2;
using CloudPtr = sensor_msgs::PointCloud2ConstPtr;
using Points = pcl::PointCloud<pcl::PointXYZI>;
using Bounds = pointcloud_fusion::BoundsConfig;
using Extrinsics = pointcloud_fusion::ExtrinsicsConfig;
using Filter = pointcloud_fusion::FilterConfig;
using BoundsServer = dynamic_reconfigure::Server<Bounds>;
using ExtrinsicsServer = dynamic_reconfigure::Server<Extrinsics>;
using FilterServer = dynamic_reconfigure::Server<Filter>;
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
    private_nh_.param("broadcast_tf", broadcast_tf_, true);
    private_nh_.param("extrinsics_ready", extrinsics_ready_, false);
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

    configureFilters();
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
    std::vector<std::string> ground_topics{
      "/left_rslidar_points", "/front_rslidar_points", "/right_rslidar_points"};
    private_nh_.getParam("ground_filter_topics", ground_topics);
    for (int i = 0; i < lidar_count_; ++i) {
      const std::string parameter = i == 0 ? "topics/parent_pc_topic" :
        "topics/child_pc_topic" + std::to_string(i);
      std::string topic;
      if (!private_nh_.getParam(parameter, topic) || topic.empty()) {
        throw std::runtime_error("Missing point cloud input parameter " + parameter);
      }
      ground_filter_[i] = std::find(ground_topics.begin(), ground_topics.end(), topic) != ground_topics.end();
      subscribers_[i].subscribe(nh_, topic, queue_size);
    }

    // Feed unused synchronizer slots from the parent in two- and three-lidar modes.
    sync_.reset(new Synchronizer(SyncPolicy(queue_size), subscribers_[0], subscribers_[1],
      subscribers_[lidar_count_ >= 3 ? 2 : 0], subscribers_[lidar_count_ == 4 ? 3 : 0]));
    sync_->setMaxIntervalDuration(ros::Duration(sync_slop));
    using namespace boost::placeholders;
    sync_->registerCallback(boost::bind(&PointCloudFusion::fuse, this, _1, _2, _3, _4));
  }

private:
  void configureFilters()
  {
    filters_ = Filter::__getDefault__();
    filters_.__fromServer__(private_nh_);
    if (!std::isfinite(filters_.max_range) || filters_.max_range <= 0.0 ||
        !std::isfinite(filters_.ground_z_threshold)) {
      throw std::runtime_error("Invalid filter range or ground threshold");
    }
    ROS_INFO("Cloud filtering %s", filters_.enable_filter ? "enabled" : "disabled");
    if (!dynamic_params_) {
      return;
    }
    Filter clamped = filters_;
    clamped.__clamp__();
    if (clamped.max_range != filters_.max_range ||
        clamped.ground_z_threshold != filters_.ground_z_threshold) {
      ROS_WARN("Filter tuning disabled: configured values are outside the slider limits");
      return;
    }
    ros::NodeHandle parameters(private_nh_, "filter");
    // Seed the service from the launch parameters on every start.
    filters_.__toServer__(parameters);
    filter_server_.reset(new FilterServer(parameters));
    ROS_INFO("Filter controls available in rqt at %s (Filter group)",
             parameters.getNamespace().c_str());
    filter_server_->setCallback([this](Filter& value, uint32_t) {
      filters_ = value;
      filters_.__toServer__(private_nh_);
      ROS_INFO("Cloud filtering %s, range=%s, ground=%s, max_range=%.3f, ground_z=%.3f",
               value.enable_filter ? "enabled" : "disabled",
               value.enable_range_filter ? "on" : "off",
               value.enable_ground_filter ? "on" : "off",
               value.max_range, value.ground_z_threshold);
    });
  }

  void filterInputCloud(Points& cloud, bool remove_ground) const
  {
    if (!filters_.enable_filter) {
      return;
    }
    Points filtered;
    filtered.points.reserve(cloud.size());
    for (const auto& point : cloud) {
      if (!finitePoint(point)) {
        continue;
      }
      if (filters_.enable_range_filter &&
          (std::abs(point.x) > filters_.max_range || std::abs(point.y) > filters_.max_range ||
           std::abs(point.z) > filters_.max_range)) {
        continue;
      }
      if (filters_.enable_ground_filter && remove_ground &&
          (point.z < filters_.ground_z_threshold || point.z > filters_.max_range)) {
        continue;
      }
      filtered.push_back(point);
    }
    cloud.swap(filtered);
  }

  bool keepFusedPoint(const pcl::PointXYZI& point) const
  {
    return !filters_.enable_filter ||
      (finitePoint(point) &&
       (!filters_.set_params_external_bounds || inside(point, external_bounds_)) &&
       (!filters_.set_params_internal_bounds || !inside(point, internal_bounds_)));
  }

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
      ROS_WARN("Online tuning disabled for %s: configured values exceed the compiled slider limits. "
               "Configured -> clamped: x=%.6f->%.6f, y=%.6f->%.6f, z=%.6f->%.6f, "
               "roll=%.6f->%.6f, pitch=%.6f->%.6f, yaw=%.6f->%.6f",
               name.c_str(), pose.x, clamped.x, pose.y, clamped.y, pose.z, clamped.z,
               pose.roll, clamped.roll, pose.pitch, clamped.pitch, pose.yaw, clamped.yaw);
      return;
    }
    auto& server = extrinsics_servers_[index];
    server.reset(new ExtrinsicsServer(parameters));
    ROS_INFO("Extrinsic controls available in rqt at %s", parameters.getNamespace().c_str());
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
      filterInputCloud(cloud, ground_filter_[i]);
      if (i > 0 && transform_points_) {
        pcl::transformPointCloud(cloud, cloud, transforms_[i - 1]);
        if (broadcast_tf_) {
          // Use the same child-to-parent matrix for the cloud and TF, as in calibration.
          const Eigen::Matrix4f& transform = transforms_[i - 1];
          const Eigen::Matrix3f rotation_matrix = transform.block(0, 0, 3, 3);
          const Eigen::Quaternionf q(rotation_matrix);
          const tf::Vector3 origin(transform(0, 3), transform(1, 3), transform(2, 3));
          const tf::Quaternion rotation(q.x(), q.y(), q.z(), q.w());
          tf_broadcaster_.sendTransform(tf::StampedTransform(
            tf::Transform(rotation, origin), header.stamp,
            header.frame_id, messages[i]->header.frame_id));
        }
      }
      for (const auto& point : cloud) {
        if (keepFusedPoint(point)) {
          fused.push_back(point);
          fused.is_dense = fused.is_dense && finitePoint(point);
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
  tf::TransformBroadcaster tf_broadcaster_;
  std::unique_ptr<Synchronizer> sync_;
  std::string expected_parent_frame_;
  int lidar_count_;
  bool transform_points_, extrinsics_ready_, dynamic_params_, broadcast_tf_;
  std::array<bool, 4> ground_filter_{};
  Filter filters_;
  std::unique_ptr<FilterServer> filter_server_;
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
