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
  this->declare_parameter("publish_rate", 1.0);
  this->declare_parameter("range_radius", 50.0);
  this->declare_parameter("voxel_leaf_size", 0.2);
  this->declare_parameter("map_frame_id", "map");
  this->declare_parameter("base_link_frame", "base_link");
  this->declare_parameter("octree_resolution", 1.0);

  pcd_file_path_ = this->get_parameter("pcd_file_path").as_string();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  range_radius_ = this->get_parameter("range_radius").as_double();
  voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
  map_frame_id_ = this->get_parameter("map_frame_id").as_string();
  base_link_frame_ = this->get_parameter("base_link_frame").as_string();
  octree_resolution_ = this->get_parameter("octree_resolution").as_double();

  // Validate parameters
  if (pcd_file_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "pcd_file_path parameter is required!");
    throw std::runtime_error("pcd_file_path parameter is required");
  }

  RCLCPP_INFO(this->get_logger(), "Parameters:");
  RCLCPP_INFO(this->get_logger(), "  pcd_file_path: %s", pcd_file_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "  publish_rate: %.2f Hz", publish_rate_);
  RCLCPP_INFO(this->get_logger(), "  range_radius: %.2f m", range_radius_);
  RCLCPP_INFO(this->get_logger(), "  voxel_leaf_size: %.3f m", voxel_leaf_size_);
  RCLCPP_INFO(this->get_logger(), "  map_frame_id: %s", map_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "  base_link_frame: %s", base_link_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  octree_resolution: %.2f m", octree_resolution_);

  // Initialize PCL components
  map_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  // Load PCD map
  loadPcdMap();

  // Build octree for efficient spatial search
  buildOctree();

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

void PcdRangePublisherNode::loadPcdMap()
{
  RCLCPP_INFO(this->get_logger(), "Loading PCD map from: %s", pcd_file_path_.c_str());

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, *map_cloud_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", pcd_file_path_.c_str());
    throw std::runtime_error("Failed to load PCD file");
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Successfully loaded PCD map with %zu points",
    map_cloud_->points.size());

  // Remove NaN points to avoid issues
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*map_cloud_, *map_cloud_, indices);

  RCLCPP_INFO(
    this->get_logger(),
    "After removing NaN: %zu points",
    map_cloud_->points.size());
}

void PcdRangePublisherNode::buildOctree()
{
  RCLCPP_INFO(this->get_logger(), "Building octree with resolution: %.2f m", octree_resolution_);

  octree_ = std::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(
    octree_resolution_);

  octree_->setInputCloud(map_cloud_);
  octree_->addPointsFromInputCloud();

  RCLCPP_INFO(this->get_logger(), "Octree built successfully");
}

void PcdRangePublisherNode::extractSurroundingPointCloud(
  const geometry_msgs::msg::TransformStamped & transform,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & output_cloud)
{
  // Get robot position
  pcl::PointXYZ search_point;
  search_point.x = transform.transform.translation.x;
  search_point.y = transform.transform.translation.y;
  search_point.z = transform.transform.translation.z;

  // Use octree for radius search - this is memory efficient
  std::vector<int> point_indices;
  std::vector<float> point_distances;

  octree_->radiusSearch(
    search_point,
    range_radius_,
    point_indices,
    point_distances);

  // Extract points within range
  pcl::PointCloud<pcl::PointXYZ>::Ptr range_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*map_cloud_, point_indices, *range_cloud);

  RCLCPP_DEBUG(
    this->get_logger(),
    "Extracted %zu points within %.2f m radius",
    range_cloud->points.size(),
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
