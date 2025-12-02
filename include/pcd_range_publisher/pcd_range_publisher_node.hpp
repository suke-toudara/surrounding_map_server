#ifndef PCD_RANGE_PUBLISHER__PCD_RANGE_PUBLISHER_NODE_HPP_
#define PCD_RANGE_PUBLISHER__PCD_RANGE_PUBLISHER_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>

namespace pcd_range_publisher
{

class PcdRangePublisherNode : public rclcpp::Node
{
public:
  explicit PcdRangePublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PcdRangePublisherNode() = default;

private:
  void loadPcdMap();
  void buildOctree();
  void publishTimerCallback();
  void extractSurroundingPointCloud(
    const geometry_msgs::msg::TransformStamped & transform,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & output_cloud);

  // Parameters
  std::string pcd_file_path_;
  double publish_rate_;
  double range_radius_;
  double voxel_leaf_size_;
  std::string map_frame_id_;
  std::string base_link_frame_;

  // ROS2 components
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // PCL components
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  std::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>> octree_;
  double octree_resolution_;
};

}  // namespace pcd_range_publisher

#endif  // PCD_RANGE_PUBLISHER__PCD_RANGE_PUBLISHER_NODE_HPP_
