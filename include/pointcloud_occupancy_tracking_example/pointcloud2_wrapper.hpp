#pragma once

#include <cstring>
#include <map>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <voxelized_geometry_tools/pointcloud_voxelization.hpp>


namespace pointcloud2_wrapper
{
using voxelized_geometry_tools::pointcloud_voxelization::PointCloudWrapper;

class PointCloud2Wrapper : public PointCloudWrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PointCloud2Wrapper(
      const sensor_msgs::PointCloud2* const cloud_ptr,
      const Eigen::Isometry3d& origin_transform)
      : cloud_ptr_(cloud_ptr), origin_transform_(origin_transform)
  {
    if (cloud_ptr_ == nullptr)
    {
      throw std::invalid_argument("cloud_ptr_ == nullptr");
    }
    // Figure out what the offset for XYZ fields in the pointcloud is
    std::map<std::string, size_t> field_offset_map;
    for (const auto& field : cloud_ptr_->fields)
    {
      field_offset_map[field.name] = static_cast<size_t>(field.offset);
    }
    const size_t x_offset = field_offset_map.at("x");
    const size_t y_offset = field_offset_map.at("y");
    const size_t z_offset = field_offset_map.at("z");
    if ((z_offset - y_offset) == 4 && (y_offset - x_offset) == 4)
    {
      xyz_offset_from_point_start_ = x_offset;
    }
    else
    {
      throw std::runtime_error("PointCloud does not have sequential float xyz");
    }
  }

  int64_t Size() const override
  {
    return static_cast<int64_t>(cloud_ptr_->width * cloud_ptr_->height);
  }

  const Eigen::Isometry3d& GetPointCloudOriginTransform() const override
  {
    return origin_transform_;
  }

  Eigen::Vector4d GetPointLocationDouble(
      const int64_t point_index) const override
  {
    return GetPointLocationFloat(point_index).cast<double>();
  }

  Eigen::Vector4f GetPointLocationFloat(
      const int64_t point_index) const override
  {
    Eigen::Vector4f point(0.0f, 0.0f, 0.0f, 1.0f);
    const size_t starting_offset = GetStartingOffsetForPointXYZ(point_index);
    memcpy(point.data(), &(cloud_ptr_->data.at(starting_offset)),
           sizeof(float) * 3);
    return point;
  }

  void CopyPointLocationIntoVectorDouble(
      const int64_t point_index, std::vector<double>& vector,
      const int64_t vector_index) const override
  {
    const Eigen::Vector4f point = GetPointLocationFloat(point_index);
    vector.at(static_cast<size_t>(vector_index) + 0) =
        static_cast<double>(point.x());
    vector.at(static_cast<size_t>(vector_index) + 1) =
        static_cast<double>(point.y());
    vector.at(static_cast<size_t>(vector_index) + 2) =
        static_cast<double>(point.z());
  }

  void CopyPointLocationIntoVectorFloat(
      const int64_t point_index, std::vector<float>& vector,
      const int64_t vector_index) const override
  {
    const size_t starting_offset = GetStartingOffsetForPointXYZ(point_index);
    memcpy(&(vector.at(vector_index)), &(cloud_ptr_->data.at(starting_offset)),
           sizeof(float) * 3);
  }

private:
  size_t GetStartingOffsetForPointXYZ(const int64_t point_index) const
  {
    const size_t starting_offset =
        (static_cast<size_t>(point_index)
         * static_cast<size_t>(cloud_ptr_->point_step))
        + xyz_offset_from_point_start_;
    return starting_offset;
  }

  const sensor_msgs::PointCloud2* const cloud_ptr_ = nullptr;
  size_t xyz_offset_from_point_start_ = 0;
  Eigen::Isometry3d origin_transform_ = Eigen::Isometry3d::Identity();
};
}  // namespace pointcloud2_wrapper
