#include <smb_highlevel_controller/SmbHighlevelController.hpp>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  std::string topic_laser;
  std::string topic_pointcloud;
  int queue_size;
  
  if(!nodeHandle.getParam("topic_laser", topic_laser)){
    ROS_ERROR("Could not find topic_laser parameter");
  }
  if(!nodeHandle.getParam("topic_pointcloud", topic_pointcloud)){
    ROS_ERROR("Could not find topic_pointcloud parameter");
  }
  if(!nodeHandle.getParam("queue_size", queue_size)){
    ROS_ERROR("Could not find queue_size parameter");
  }
  subscriber_laser_ = nodeHandle.subscribe(topic_laser, queue_size, &SmbHighlevelController::topic_laser_Callback, this);
  subscriber_pointcloud_ = nodeHandle.subscribe(topic_pointcloud, queue_size, &SmbHighlevelController::topic_pointcloud_Callback, this);
}

SmbHighlevelController::~SmbHighlevelController()
{
}

void SmbHighlevelController::topic_laser_Callback(const sensor_msgs::LaserScan data)
{
  int i = 0;
  int size = data.ranges.size();

  float min_dis = 10000;

  for (i = 0; i < size; i++)
  {
    float dist = data.ranges.at(i);

    if (dist < min_dis) min_dis = dist;
  }
  ROS_INFO("min distance: %f", min_dis);
}

void SmbHighlevelController::topic_pointcloud_Callback(const sensor_msgs::PointCloud2 data)
{
  int size = data.height * data.width;
  ROS_INFO("Pointcloud size: %i", size);
}
} /* namespace */
