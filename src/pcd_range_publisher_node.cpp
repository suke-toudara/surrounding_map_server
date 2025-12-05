#include "pcd_range_publisher/pcd_range_publisher_node.hpp"

#include <chrono>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pcd_range_publisher
{

PcdRangePublisherNode::PcdRangePublisherNode(const rclcpp::NodeOptions & options)
: Node("pcd_range_publisher", options)
{
  // Declare and get parameters
  this->declare_parameter("pcd_file_path", "");
  this->declare_parameter("publish_rate", 0.1);  // 10 seconds interval (0.1 Hz)
  this->declare_parameter("range_radius", 100.0);  // 100 meters
  this->declare_parameter("voxel_leaf_size", 0.2);
  this->declare_parameter("map_frame_id", "map");
  this->declare_parameter("base_link_frame", "base_link");
  this->declare_parameter("chunk_size", 10000);  // Points per chunk

  pcd_file_path_ = this->get_parameter("pcd_file_path").as_string();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  range_radius_ = this->get_parameter("range_radius").as_double();
  voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
  map_frame_id_ = this->get_parameter("map_frame_id").as_string();
  base_link_frame_ = this->get_parameter("base_link_frame").as_string();
  chunk_size_ = this->get_parameter("chunk_size").as_int();

  // Validate parameters
  if (pcd_file_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "pcd_file_path parameter is required!");
    throw std::runtime_error("pcd_file_path parameter is required");
  }

  RCLCPP_INFO(this->get_logger(), "Parameters:");
  RCLCPP_INFO(this->get_logger(), "  pcd_file_path: %s", pcd_file_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "  publish_rate: %.2f Hz (%.1f seconds interval)",
    publish_rate_, 1.0 / publish_rate_);
  RCLCPP_INFO(this->get_logger(), "  range_radius: %.2f m", range_radius_);
  RCLCPP_INFO(this->get_logger(), "  voxel_leaf_size: %.3f m", voxel_leaf_size_);
  RCLCPP_INFO(this->get_logger(), "  map_frame_id: %s", map_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "  base_link_frame: %s", base_link_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  chunk_size: %d points", chunk_size_);

  // Initialize memory-mapped PCD reader with chunk-based reading
  RCLCPP_INFO(this->get_logger(), "Initializing memory-mapped PCD reader (chunk-based)...");
  try {
    pcd_reader_ = std::make_unique<PcdMmapReader>(pcd_file_path_, chunk_size_);
    RCLCPP_INFO(this->get_logger(),
      "Successfully mapped PCD file with %zu points using mmap() - memory efficient mode",
      pcd_reader_->getTotalPoints());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize PCD reader: %s", e.what());
    throw;
  }

  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create publisher
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "surrounding_pointcloud", 10);

  // Create timer for periodic publishing
  auto timer_interval = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(timer_interval),
    std::bind(&PcdRangePublisherNode::publishTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "PCD Range Publisher initialized successfully");
}


void PcdRangePublisherNode::extractSurroundingPointCloud(
  const geometry_msgs::msg::TransformStamped & transform,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & output_cloud)
{
  // Get robot position
  float center_x = transform.transform.translation.x;
  float center_y = transform.transform.translation.y;
  float center_z = transform.transform.translation.z;

  // Use mmap-based reader to get points within range
  pcl::PointCloud<pcl::PointXYZ>::Ptr range_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  size_t num_points = pcd_reader_->getPointsInRange(
    center_x, center_y, center_z,
    range_radius_,
    range_cloud);

  RCLCPP_DEBUG(
    this->get_logger(),
    "Extracted %zu points within %.2f m radius using mmap()",
    num_points,
    range_radius_);

  // Apply voxel grid filter for resolution control
  if (voxel_leaf_size_ > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(range_cloud);
    voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_filter.filter(*output_cloud);

    RCLCPP_DEBUG(
      this->get_logger(),
      "After voxel filtering: %zu points (leaf size: %.3f m)",
      output_cloud->points.size(),
      voxel_leaf_size_);
  } else {
    *output_cloud = *range_cloud;
  }
}

void PcdRangePublisherNode::publishTimerCallback()
{
  try {
    // Get transform from map to base_link
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf_buffer_->lookupTransform(
      map_frame_id_,
      base_link_frame_,
      tf2::TimePointZero);

    // Extract surrounding pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr surrounding_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extractSurroundingPointCloud(transform_stamped, surrounding_cloud);

    // Convert to ROS message
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*surrounding_cloud, output_msg);
    output_msg.header.frame_id = map_frame_id_;
    output_msg.header.stamp = this->now();

    // Publish
    pointcloud_pub_->publish(output_msg);

    RCLCPP_DEBUG(
      this->get_logger(),
      "Published pointcloud with %zu points at position (%.2f, %.2f, %.2f)",
      surrounding_cloud->points.size(),
      transform_stamped.transform.translation.x,
      transform_stamped.transform.translation.y,
      transform_stamped.transform.translation.z);

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,  // Log every 5 seconds
      "Could not get transform from %s to %s: %s",
      map_frame_id_.c_str(),
      base_link_frame_.c_str(),
      ex.what());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(this->get_logger(), "Error in publish callback: %s", ex.what());
  }
}

}  // namespace pcd_range_publisher

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcd_range_publisher::PcdRangePublisherNode)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<pcd_range_publisher::PcdRangePublisherNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("pcd_range_publisher"), "Exception: %s", ex.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
