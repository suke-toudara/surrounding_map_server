#ifndef PCD_RANGE_PUBLISHER__PCD_MMAP_READER_HPP_
#define PCD_RANGE_PUBLISHER__PCD_MMAP_READER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <cstdint>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>

namespace pcd_range_publisher
{

/**
 * @brief Memory-mapped PCD file reader for efficient point cloud access
 *
 * This class uses mmap() to map PCD files into memory without loading all data at once.
 * It builds an octree index for efficient spatial queries and only loads point data
 * when needed.
 */
class PcdMmapReader
{
public:
  /**
   * @brief Constructor
   * @param file_path Path to the PCD file
   * @param octree_resolution Resolution for the octree spatial index
   */
  PcdMmapReader(const std::string & file_path, double octree_resolution);

  /**
   * @brief Destructor - unmaps the file
   */
  ~PcdMmapReader();

  /**
   * @brief Get points within a specified radius from a center point
   * @param center_x X coordinate of center point
   * @param center_y Y coordinate of center point
   * @param center_z Z coordinate of center point
   * @param radius Search radius in meters
   * @param output_cloud Output point cloud containing points within range
   * @return Number of points found
   */
  size_t getPointsInRange(
    float center_x,
    float center_y,
    float center_z,
    float radius,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & output_cloud);

  /**
   * @brief Get the total number of points in the PCD file
   * @return Total number of points
   */
  size_t getTotalPoints() const { return num_points_; }

  /**
   * @brief Check if the file was successfully opened
   * @return true if file is ready
   */
  bool isValid() const { return mapped_data_ != nullptr; }

private:
  /**
   * @brief Parse PCD header to extract metadata
   */
  void parseHeader();

  /**
   * @brief Build octree index for spatial queries
   */
  void buildOctreeIndex();

  /**
   * @brief Read a single point from the mapped memory
   * @param index Point index
   * @param point Output point
   * @return true if successful
   */
  bool readPoint(size_t index, pcl::PointXYZ & point);

  // File information
  std::string file_path_;
  int fd_;                    // File descriptor
  void * mapped_data_;        // Memory-mapped data pointer
  size_t file_size_;          // Total file size
  size_t data_offset_;        // Offset to point data in file

  // PCD format information
  enum DataFormat { ASCII, BINARY, BINARY_COMPRESSED };
  DataFormat data_format_;
  size_t num_points_;
  size_t point_step_;         // Bytes per point

  // Field information
  struct FieldInfo {
    std::string name;
    size_t offset;
    char type;      // 'F' = float, 'U' = uint, 'I' = int
    size_t size;
  };
  std::vector<FieldInfo> fields_;
  int x_field_idx_;
  int y_field_idx_;
  int z_field_idx_;

  // Octree for spatial indexing
  pcl::PointCloud<pcl::PointXYZ>::Ptr index_cloud_;  // Only stores coordinates for octree
  std::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>> octree_;
  double octree_resolution_;
};

}  // namespace pcd_range_publisher

#endif  // PCD_RANGE_PUBLISHER__PCD_MMAP_READER_HPP_
