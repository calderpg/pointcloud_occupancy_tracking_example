#include <cstring>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/ros_conversions.hpp>
#include <common_robotics_utilities/color_builder.hpp>
#include <common_robotics_utilities/math.hpp>
#include <ros/ros.h>
#include <pointcloud_occupancy_tracking_example/MultiPointCloud2.h>
#include <pointcloud_occupancy_tracking_example/pointcloud2_wrapper.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxelized_geometry_tools/collision_map.hpp>
#include <voxelized_geometry_tools/pointcloud_voxelization.hpp>
#include <voxelized_geometry_tools/ros_interface.hpp>

using voxelized_geometry_tools::pointcloud_voxelization
    ::PointCloudVoxelizationFilterOptions;
using voxelized_geometry_tools::pointcloud_voxelization
    ::PointCloudVoxelizationInterface;
using voxelized_geometry_tools::pointcloud_voxelization::PointCloudWrapper;
using voxelized_geometry_tools::pointcloud_voxelization::PointCloudWrapperPtr;
using voxelized_geometry_tools::pointcloud_voxelization::VoxelizerOptions;

using pointcloud2_wrapper::PointCloud2Wrapper;

class OccupancyTracker
{
public:
  OccupancyTracker(
      const ros::NodeHandle& nh,
      const common_robotics_utilities::voxel_grid::GridSizes& grid_sizes,
      const Eigen::Isometry3d& grid_origin_transform,
      const std::string& parent_frame_name, const double step_size_multiplier,
      const PointCloudVoxelizationFilterOptions& filter_options,
      const VoxelizerOptions voxelizer_option,
      const std::map<std::string, int32_t>& options,
      const std::string& pointclouds_topic,
      const std::string& occupancy_display_topic)
      : nh_(nh), step_size_multiplier_(step_size_multiplier),
        filter_options_(filter_options)
  {
    const voxelized_geometry_tools::CollisionCell default_cell(0.0f);
    static_environment_ = voxelized_geometry_tools::CollisionMap(
        grid_origin_transform, parent_frame_name, grid_sizes, default_cell);
    voxelizer_ = voxelized_geometry_tools::pointcloud_voxelization
                     ::MakePointCloudVoxelizer(voxelizer_option, options);
    display_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
                       occupancy_display_topic, 1, false);
    pointclouds_sub_ =
        nh_.subscribe(pointclouds_topic, 1,
                      &OccupancyTracker::PointCloudsCallback, this);
  }

  void Loop(const double frequency)
  {
    ROS_INFO_NAMED(ros::this_node::getName(), "Occupancy tracker running");
    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_INFO_NAMED(ros::this_node::getName(), "Occupancy tracker shut down");
  }

  void PointCloudsCallback(
      const pointcloud_occupancy_tracking_example::MultiPointCloud2& msg)
  {
    if (msg.pointclouds.size() != msg.camera_poses.size())
    {
      ROS_WARN_NAMED(
          ros::this_node::getName(),
          "Number of clouds and number of camera poses do not match!");
      return;
    }
    if (msg.header.frame_id != static_environment_.GetFrame())
    {
      ROS_WARN_NAMED(
          ros::this_node::getName(),
          "Message frame does not match static environment frame");
      return;
    }
    ROS_INFO_NAMED(ros::this_node::getName(), "Got %zu new clouds",
                   msg.pointclouds.size());
    std::vector<PointCloudWrapperPtr> clouds;
    for (size_t idx = 0; idx < msg.pointclouds.size(); idx++)
    {
      clouds.push_back(PointCloudWrapperPtr(
          new PointCloud2Wrapper(
              &(msg.pointclouds.at(idx)),
              common_robotics_utilities::ros_conversions
                  ::GeometryPoseToEigenIsometry3d(msg.camera_poses.at(idx)))));
    }
    const auto voxelized = voxelizer_->VoxelizePointClouds(
        static_environment_, step_size_multiplier_, filter_options_, clouds);
    // Draw
    const std_msgs::ColorRGBA free_color
        = common_robotics_utilities::color_builder
            ::MakeFromFloatColors<std_msgs::ColorRGBA>(
                0.0, 0.25, 0.0, 0.5);
    const std_msgs::ColorRGBA filled_color
        = common_robotics_utilities::color_builder
            ::MakeFromFloatColors<std_msgs::ColorRGBA>(
                0.25, 0.0, 0.0, 0.5);
    const std_msgs::ColorRGBA unknown_color
        = common_robotics_utilities::color_builder
            ::MakeFromFloatColors<std_msgs::ColorRGBA>(
                0.0, 0.0, 0.25, 0.5);
    const auto environment_display =
        voxelized_geometry_tools::ros_interface
            ::ExportForSeparateDisplay(
                voxelized, filled_color, free_color, unknown_color);
    display_pub_.publish(environment_display);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber pointclouds_sub_;
  ros::Publisher display_pub_;
  voxelized_geometry_tools::CollisionMap static_environment_;
  double step_size_multiplier_ = 0.5;
  PointCloudVoxelizationFilterOptions filter_options_;
  std::unique_ptr<PointCloudVoxelizationInterface> voxelizer_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_occupancy_tracking_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Load parameters
  // We require that 100% of points from the camera see through to see a voxel
  // as free.
  const double percent_seen_free =
      nhp.param(std::string("percent_seen_free"), 1.0);
  // We don't worry about outliers
  const int32_t outlier_points_threshold =
      nhp.param(std::string("outlier_points_threshold"), 1);
  // We only need one camera to see a voxel as free.
  const int32_t num_cameras_seen_free =
      nhp.param(std::string("num_cameras_seen_free"), 1);
  const PointCloudVoxelizationFilterOptions filter_options(
      percent_seen_free, outlier_points_threshold, num_cameras_seen_free);
  const double step_size_multiplier =
      nhp.param(std::string("step_size_multiplier"), 0.5);
  const std::string raw_voxelizer_option =
      nhp.param(std::string("voxelizer_option"), std::string("best"));
  VoxelizerOptions voxelizer_option = VoxelizerOptions::BEST_AVAILABLE;
  if (raw_voxelizer_option == "best")
  {
    voxelizer_option = VoxelizerOptions::BEST_AVAILABLE;
  }
  else if (raw_voxelizer_option == "cpu")
  {
    voxelizer_option = VoxelizerOptions::CPU;
  }
  else if (raw_voxelizer_option == "opencl")
  {
    voxelizer_option = VoxelizerOptions::OPENCL;
  }
  else if (raw_voxelizer_option == "cuda")
  {
    voxelizer_option = VoxelizerOptions::CUDA;
  }
  else
  {
    ROS_FATAL_NAMED(ros::this_node::getName(),
                    "[%s] is not a valid voxelizer option",
                    raw_voxelizer_option.c_str());
  }
  std::map<std::string, int32_t> options;
  options["CUDA_DEVICE"] = nhp.param(std::string("cuda_device"), 0);
  options["OPENCL_PLATFORM_INDEX"] =
      nhp.param(std::string("opencl_platform_index"), 0);
  options["OPENCL_DEVICE_INDEX"] =
      nhp.param(std::string("opencl_device_index"), 0);
  // Make grid sizes
  const double grid_resolution =
      nhp.param(std::string("grid_resolution"), 0.04);
  const double x_size = nhp.param(std::string("x_size"), 2.0);
  const double y_size = nhp.param(std::string("y_size"), 2.0);
  const double z_size = nhp.param(std::string("z_size"), 2.0);
  const common_robotics_utilities::voxel_grid::GridSizes grid_sizes(
    grid_resolution, x_size, y_size, z_size);
  // Frame name
  const std::string parent_frame_name =
      nhp.param(std::string("parent_frame_name"), std::string("world"));
  const double x_origin = nhp.param(std::string("x_origin"), -1.0);
  const double y_origin = nhp.param(std::string("y_origin"), -1.0);
  const double z_origin = nhp.param(std::string("z_origin"), 0.0);
  const Eigen::Isometry3d grid_origin_transform(
      Eigen::Translation3d(x_origin, y_origin, z_origin));
  // Topics
  const std::string pointclouds_topic =
      nhp.param(std::string("pointclouds_topic"),
                std::string("combined_pointclouds"));
  const std::string occupancy_display_topic =
      nhp.param(std::string("occupancy_display_topic"),
                std::string("occupancy_display"));
  // Loop rate
  const double loop_rate = nhp.param(std::string("loop_rate"), 30.0);
  // Start
  OccupancyTracker tracker(
      nh, grid_sizes, grid_origin_transform, parent_frame_name,
      step_size_multiplier, filter_options, voxelizer_option, options,
      pointclouds_topic, occupancy_display_topic);
  tracker.Loop(loop_rate);
  return 0;
}
