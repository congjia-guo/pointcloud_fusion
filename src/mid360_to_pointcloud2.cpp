#include <cmath>
#include <string>

#include <livox_ros_driver2/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class Mid360Converter
{
public:
  Mid360Converter()
  {
    ros::NodeHandle private_nh("~");
    std::string input_topic, output_topic;
    private_nh.param<std::string>("input_topic", input_topic, "/livox/lidar");
    private_nh.param<std::string>("output_topic", output_topic, "/livox/lidar_pointcloud2");
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic, 10);
    subscriber_ = nh_.subscribe(input_topic, 10, &Mid360Converter::convert, this);
  }

private:
  void convert(const livox_ros_driver2::CustomMsg::ConstPtr& input)
  {
    sensor_msgs::PointCloud2 cloud;
    cloud.header = input->header;
    cloud.height = 1;
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(input->points.size());

    if (!input->points.empty()) {
      sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
      sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
      for (const auto& point : input->points) {
        *x = point.x;
        *y = point.y;
        *z = point.z;
        *intensity = static_cast<float>(point.reflectivity);
        cloud.is_dense &= std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        ++x;
        ++y;
        ++z;
        ++intensity;
      }
    }
    publisher_.publish(cloud);
  }

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mid360_to_pointcloud2");
  Mid360Converter converter;
  ros::spin();
}
