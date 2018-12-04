#include <map>
#include <string>

#include <Eigen/Geometry>
#include <common_robotics_utilities/conversions.hpp>
#include <pointcloud_occupancy_tracking_example/MultiPointCloud2.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>

using common_robotics_utilities::conversions::TransformFromUrdfXYZRPY;

std::map<std::string, sensor_msgs::PointCloud2> g_current_pointclouds;

void EnqueuePointCloud2(const sensor_msgs::PointCloud2& cloud)
{
  g_current_pointclouds[cloud.header.frame_id] = cloud;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_aggregator_node");
  // Get a handle to the current node
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Get parameters
  const std::string combined_pointcloud_topic =
      nhp.param(std::string("pointclouds_topic"),
                std::string("combined_pointclouds"));
  // Make subscribers for pointclouds
  const std::vector<std::string> pointcloud_topics =
      {"/cam1/depth/points", "/cam2/depth/points", "/cam3/depth/points",
       "/cam4/depth/points", "/cam5/depth/points", "/cam6/depth/points",
       "/cam7/depth/points", "/cam8/depth/points"};
  std::vector<ros::Subscriber> pointcloud_subs;
  for (size_t idx = 0; idx < pointcloud_topics.size(); idx++)
  {
    pointcloud_subs.push_back(
        nh.subscribe(pointcloud_topics.at(idx), 1, EnqueuePointCloud2));
  }
  // Define the camera poses
  // Make the physical->optical frame transform
  const Eigen::Isometry3d X_CO = Eigen::Translation3d(0.0, 0.0, 0.0) *
      Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()));
  common_robotics_utilities::math::MapStringIsometry3d camera_poses;
  camera_poses["cam1_link"] =
      TransformFromUrdfXYZRPY(-1.0, -1.0, 0.0, 0.0, -0.785398, 0.785398) * X_CO;
  camera_poses["cam2_link"] =
      TransformFromUrdfXYZRPY(-1.0, 1.0, 0.0, 0.0, -0.785398, -0.785398) * X_CO;
  camera_poses["cam3_link"] =
      TransformFromUrdfXYZRPY(1.0, -1.0, 0.0, 0.0, -0.785398, 2.35619) * X_CO;
  camera_poses["cam4_link"] =
      TransformFromUrdfXYZRPY(1.0, 1.0, 0.0, 0.0, -0.785398, -2.35619) * X_CO;
  camera_poses["cam5_link"] =
      TransformFromUrdfXYZRPY(-1.0, -1.0, 2.0, 0.0, 0.785398, 0.785398) * X_CO;
  camera_poses["cam6_link"] =
      TransformFromUrdfXYZRPY(-1.0, 1.0, 2.0, 0.0, 0.785398, -0.785398) * X_CO;
  camera_poses["cam7_link"] =
      TransformFromUrdfXYZRPY(1.0, -1.0, 2.0, 0.0, 0.785398, 2.35619) * X_CO;
  camera_poses["cam8_link"] =
      TransformFromUrdfXYZRPY(1.0, 1.0, 2.0, 0.0, 0.785398, -2.35619) * X_CO;
  // Make a publisher for combined pointclouds
  ros::Publisher pointcloud_publisher =
      nh.advertise<pointcloud_occupancy_tracking_example::MultiPointCloud2>(
          combined_pointcloud_topic, 1, false);
  ros::Publisher tf_publisher =
      nh.advertise<tf2_msgs::TFMessage>("/tf", 1, false);
  // Loop
  ros::Rate loop_rate(30.0);
  while (ros::ok())
  {
    ros::spinOnce();
    const ros::Time now_time = ros::Time::now();
    // Attempt to publish the pointclouds together
    pointcloud_occupancy_tracking_example::MultiPointCloud2
        combined_pointclouds;
    combined_pointclouds.header.stamp = now_time;
    combined_pointclouds.header.frame_id = "world";
    tf2_msgs::TFMessage camera_transforms;
    // Go through the current pointclouds
    for (auto itr = g_current_pointclouds.begin();
         itr != g_current_pointclouds.end(); ++itr)
    {
      combined_pointclouds.pointclouds.push_back(itr->second);
      combined_pointclouds.camera_poses.push_back(
          common_robotics_utilities::conversions
              ::EigenIsometry3dToGeometryPose(camera_poses.at(itr->first)));
      camera_transforms.transforms.push_back(
          common_robotics_utilities::conversions
              ::EigenIsometry3dToGeometryTransformStamped(
                  camera_poses.at(itr->first), "world", itr->first));
      camera_transforms.transforms.back().header.stamp = now_time;
    }
    pointcloud_publisher.publish(combined_pointclouds);
    tf_publisher.publish(camera_transforms);
    loop_rate.sleep();
  }
  return 0;
}
