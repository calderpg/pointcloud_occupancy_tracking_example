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

  int64_t Size() const override
  {
    return static_cast<int64_t>(cloud_ptr_->width * cloud_ptr_->height);
  }

  const Eigen::Isometry3d& GetPointCloudOriginTransform() const override
  {
    return origin_transform_;
  }

  void SetPointCloudOriginTransform(
      const Eigen::Isometry3d& origin_transform) override
  {
    origin_transform_ = origin_transform;
  }

protected:
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

private:
  void CopyPointLocationIntoDoublePtrImpl(
      const int64_t point_index, double* destination) const override
  {
    const Eigen::Vector4d point =
        GetPointLocationVector4f(point_index).cast<double>();
    std::memcpy(destination, point.data(), sizeof(double) * 3);
  }

  void CopyPointLocationIntoFloatPtrImpl(
      const int64_t point_index, float* destination) const override
  {
    const size_t starting_offset = GetStartingOffsetForPointXYZ(point_index);
    std::memcpy(destination, &(cloud_ptr_->data.at(starting_offset)),
                sizeof(float) * 3);
  }

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

class NonOwningPointCloud2Wrapper : public PointCloud2Wrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NonOwningPointCloud2Wrapper(
      const sensor_msgs::PointCloud2* const cloud_ptr,
      const Eigen::Isometry3d& origin_transform)
      : PointCloud2Wrapper(cloud_ptr, origin_transform) {}
};

class OwningPointCloud2Wrapper : public PointCloud2Wrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OwningPointCloud2Wrapper(
      const sensor_msgs::PointCloud2ConstPtr& cloud_ptr,
      const Eigen::Isometry3d& origin_transform)
      : PointCloud2Wrapper(cloud_ptr.get(), origin_transform),
        owned_cloud_ptr_(cloud_ptr) {}

private:
  sensor_msgs::PointCloud2ConstPtr owned_cloud_ptr_;
};
}  // namespace pointcloud2_wrapper
