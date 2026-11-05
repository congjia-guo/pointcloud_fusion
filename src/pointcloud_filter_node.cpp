#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloudFilter
{
public:
  PointCloudFilter()
  {
    ros::NodeHandle private_nh("~");
    private_nh.param("max_range", max_range_, 5.0);
    private_nh.param("ground_z_threshold", ground_z_threshold_, 0.0);
    private_nh.param("enable_range_filter", enable_range_, true);
    private_nh.param("enable_ground_filter", enable_ground_, true);
    if (!std::isfinite(max_range_) || max_range_ <= 0.0 || !std::isfinite(ground_z_threshold_)) {
      throw std::runtime_error("Invalid filter range or ground threshold");
    }

    std::vector<std::string> inputs, outputs, ground_topics;
    if (!private_nh.getParam("input_topics", inputs) || inputs.size() != channels_.size() ||
        !private_nh.getParam("output_topics", outputs) || outputs.size() != channels_.size()) {
      throw std::runtime_error("input_topics and output_topics must each contain four topics");
    }
    private_nh.getParam("ground_filter_topics", ground_topics);
    for (std::size_t i = 0; i < channels_.size(); ++i) {
      auto& channel = channels_[i];
      channel.remove_ground = enable_ground_ &&
        std::find(ground_topics.begin(), ground_topics.end(), inputs[i]) != ground_topics.end();
      channel.publisher = nh_.advertise<sensor_msgs::PointCloud2>(outputs[i], 10);
      channel.subscriber = nh_.subscribe<sensor_msgs::PointCloud2>(inputs[i], 10,
        [this, i](const sensor_msgs::PointCloud2ConstPtr& cloud) { filter(cloud, channels_[i]); });
    }
  }

private:
  struct Channel
  {
    ros::Publisher publisher;
    ros::Subscriber subscriber;
    bool remove_ground;
  };

  void filter(const sensor_msgs::PointCloud2ConstPtr& input, const Channel& channel) const
  {
    pcl::PointCloud<pcl::PointXYZI> source, filtered;
    if (input->width != 0 && input->height != 0) {
      pcl::fromROSMsg(*input, source);
    }
    filtered.points.reserve(source.size());
    for (const auto& point : source) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (enable_range_ && (std::abs(point.x) > max_range_ || std::abs(point.y) > max_range_ ||
                            std::abs(point.z) > max_range_)) {
        continue;
      }
      if (channel.remove_ground && (point.z < ground_z_threshold_ || point.z > max_range_)) {
        continue;
      }
      filtered.push_back(point);
    }
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(filtered, output);
    output.header = input->header;
    channel.publisher.publish(output);
  }

  ros::NodeHandle nh_;
  std::array<Channel, 4> channels_;
  double max_range_, ground_z_threshold_;
  bool enable_range_, enable_ground_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_filter_node");
  try {
    PointCloudFilter filter;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("%s", error.what());
    return 1;
  }
}
