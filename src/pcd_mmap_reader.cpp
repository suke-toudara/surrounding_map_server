#include "pcd_range_publisher/pcd_mmap_reader.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <cstring>
#include <sstream>
#include <stdexcept>
#include <algorithm>

namespace pcd_range_publisher
{

PcdMmapReader::PcdMmapReader(const std::string & file_path, double octree_resolution)
: file_path_(file_path),
  fd_(-1),
  mapped_data_(nullptr),
  file_size_(0),
  data_offset_(0),
  data_format_(BINARY),
  num_points_(0),
  point_step_(0),
  x_field_idx_(-1),
  y_field_idx_(-1),
  z_field_idx_(-1),
  octree_resolution_(octree_resolution)
{
  // Open file
  fd_ = open(file_path_.c_str(), O_RDONLY);
  if (fd_ == -1) {
    throw std::runtime_error("Failed to open PCD file: " + file_path_);
  }

  // Get file size
  struct stat sb;
  if (fstat(fd_, &sb) == -1) {
    close(fd_);
    throw std::runtime_error("Failed to get file size: " + file_path_);
  }
  file_size_ = sb.st_size;

  // Memory map the file
  mapped_data_ = mmap(nullptr, file_size_, PROT_READ, MAP_PRIVATE, fd_, 0);
  if (mapped_data_ == MAP_FAILED) {
    close(fd_);
    throw std::runtime_error("Failed to mmap file: " + file_path_);
  }

  // Parse header to get metadata
  parseHeader();

  // Build octree index for efficient spatial queries
  buildOctreeIndex();
}

PcdMmapReader::~PcdMmapReader()
{
  if (mapped_data_ != nullptr && mapped_data_ != MAP_FAILED) {
    munmap(mapped_data_, file_size_);
  }
  if (fd_ != -1) {
    close(fd_);
  }
}

void PcdMmapReader::parseHeader()
{
  const char * data = static_cast<const char *>(mapped_data_);
  size_t pos = 0;

  // Parse header line by line
  std::string line;
  while (pos < file_size_) {
    // Read line
    line.clear();
    while (pos < file_size_ && data[pos] != '\n') {
      line += data[pos++];
    }
    pos++;  // Skip newline

    // Trim trailing whitespace
    while (!line.empty() && std::isspace(line.back())) {
      line.pop_back();
    }

    if (line.empty() || line[0] == '#') {
      continue;  // Skip empty lines and comments
    }

    // Parse header fields
    std::istringstream iss(line);
    std::string key;
    iss >> key;

    if (key == "FIELDS") {
      // Parse field names
      std::string field_name;
      while (iss >> field_name) {
        FieldInfo field;
        field.name = field_name;
        fields_.push_back(field);
      }
    } else if (key == "SIZE") {
      // Parse field sizes
      size_t i = 0;
      int size;
      while (iss >> size && i < fields_.size()) {
        fields_[i++].size = size;
      }
    } else if (key == "TYPE") {
      // Parse field types
      size_t i = 0;
      char type;
      while (iss >> type && i < fields_.size()) {
        fields_[i++].type = type;
      }
    } else if (key == "POINTS") {
      iss >> num_points_;
    } else if (key == "DATA") {
      std::string format;
      iss >> format;
      if (format == "ascii") {
        data_format_ = ASCII;
      } else if (format == "binary") {
        data_format_ = BINARY;
      } else if (format == "binary_compressed") {
        data_format_ = BINARY_COMPRESSED;
        throw std::runtime_error("Binary compressed PCD format is not supported");
      }

      // Data starts on next line
      data_offset_ = pos;
      break;
    }
  }

  // Calculate field offsets and find x, y, z fields
  size_t offset = 0;
  for (size_t i = 0; i < fields_.size(); ++i) {
    fields_[i].offset = offset;
    offset += fields_[i].size;

    if (fields_[i].name == "x") {
      x_field_idx_ = i;
    } else if (fields_[i].name == "y") {
      y_field_idx_ = i;
    } else if (fields_[i].name == "z") {
      z_field_idx_ = i;
    }
  }
  point_step_ = offset;

  // Validate that we found x, y, z fields
  if (x_field_idx_ == -1 || y_field_idx_ == -1 || z_field_idx_ == -1) {
    throw std::runtime_error("PCD file must contain x, y, z fields");
  }
}

void PcdMmapReader::buildOctreeIndex()
{
  // Create point cloud to store only coordinates for octree
  index_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  index_cloud_->points.reserve(num_points_);

  // Read all point coordinates
  for (size_t i = 0; i < num_points_; ++i) {
    pcl::PointXYZ point;
    if (readPoint(i, point)) {
      // Skip invalid points
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        index_cloud_->points.push_back(point);
      }
    }
  }

  index_cloud_->width = index_cloud_->points.size();
  index_cloud_->height = 1;
  index_cloud_->is_dense = false;

  // Build octree
  octree_ = std::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(
    octree_resolution_);
  octree_->setInputCloud(index_cloud_);
  octree_->addPointsFromInputCloud();
}

bool PcdMmapReader::readPoint(size_t index, pcl::PointXYZ & point)
{
  if (index >= num_points_) {
    return false;
  }

  if (data_format_ == BINARY) {
    // Binary format - direct memory access
    const char * point_data = static_cast<const char *>(mapped_data_) +
      data_offset_ + (index * point_step_);

    // Read x, y, z fields
    const float * x_ptr = reinterpret_cast<const float *>(
      point_data + fields_[x_field_idx_].offset);
    const float * y_ptr = reinterpret_cast<const float *>(
      point_data + fields_[y_field_idx_].offset);
    const float * z_ptr = reinterpret_cast<const float *>(
      point_data + fields_[z_field_idx_].offset);

    point.x = *x_ptr;
    point.y = *y_ptr;
    point.z = *z_ptr;

    return true;
  } else if (data_format_ == ASCII) {
    // ASCII format - parse text
    const char * data = static_cast<const char *>(mapped_data_) + data_offset_;
    size_t line_count = 0;
    size_t pos = 0;
    size_t data_size = file_size_ - data_offset_;

    // Skip to the correct line
    while (line_count < index && pos < data_size) {
      if (data[pos++] == '\n') {
        line_count++;
      }
    }

    if (line_count != index || pos >= data_size) {
      return false;
    }

    // Read the line
    std::string line;
    while (pos < data_size && data[pos] != '\n') {
      line += data[pos++];
    }

    // Parse values
    std::istringstream iss(line);
    std::vector<float> values;
    float value;
    while (iss >> value) {
      values.push_back(value);
    }

    if (values.size() < fields_.size()) {
      return false;
    }

    point.x = values[x_field_idx_];
    point.y = values[y_field_idx_];
    point.z = values[z_field_idx_];

    return true;
  }

  return false;
}

size_t PcdMmapReader::getPointsInRange(
  float center_x,
  float center_y,
  float center_z,
  float radius,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & output_cloud)
{
  if (!isValid()) {
    return 0;
  }

  // Create search point
  pcl::PointXYZ search_point(center_x, center_y, center_z);

  // Use octree for radius search
  std::vector<int> point_indices;
  std::vector<float> point_distances;

  octree_->radiusSearch(search_point, radius, point_indices, point_distances);

  // Extract points using the indices
  output_cloud->points.clear();
  output_cloud->points.reserve(point_indices.size());

  for (int idx : point_indices) {
    if (idx >= 0 && static_cast<size_t>(idx) < index_cloud_->points.size()) {
      // We can directly use the point from index_cloud since we only store valid points
      output_cloud->points.push_back(index_cloud_->points[idx]);
    }
  }

  output_cloud->width = output_cloud->points.size();
  output_cloud->height = 1;
  output_cloud->is_dense = false;

  return output_cloud->points.size();
}

}  // namespace pcd_range_publisher
