#include <smb_highlevel_controller/SmbHighlevelController.hpp>
#include <sensor_msgs/LaserScan.h>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  std::string topic;
  int queue_size;
  
  if(!nodeHandle.getParam("topic", topic)){
    ROS_ERROR("Could not find topic parameter");
  }
  if(!nodeHandle.getParam("queue_size", queue_size)){
    ROS_ERROR("Could not find queue_size parameter");
  }
  subscriber_ = nodeHandle.subscribe(topic, queue_size, &SmbHighlevelController::topicCallback, this);
}

SmbHighlevelController::~SmbHighlevelController()
{
}

void SmbHighlevelController::topicCallback(const sensor_msgs::LaserScan data)
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

} /* namespace */
