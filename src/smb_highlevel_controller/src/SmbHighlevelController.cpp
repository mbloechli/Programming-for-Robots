#include <smb_highlevel_controller/SmbHighlevelController.hpp>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

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
  publisher_twist_ = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  publisher_rvizMarker_ = nodeHandle.advertise<visualization_msgs::Marker>( "/visualization_marker", 0 );

  subscriber_laser_ = nodeHandle.subscribe(topic_laser, queue_size, &SmbHighlevelController::topic_laser_Callback, this);
  //subscriber_pointcloud_ = nodeHandle.subscribe(topic_pointcloud, queue_size, &SmbHighlevelController::topic_pointcloud_Callback, this);

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
  //ROS_INFO("min distance: %f", min_dis);
  // Find the pillar
  for (i = 0; data.ranges.at(i) > 50; i++)  {} // look for pillar from left side
  float angle = data.angle_min + i * data.angle_increment;
  
  for (i = data.ranges.size()-1; data.ranges.at(i) > 50; i--)  {} // look for pillar from right side
  angle = (angle + data.angle_min + i * data.angle_increment) / 2; 
  ROS_INFO("Pillar found at %i and angle %f", i, angle);

  // Publish velocity commands
  float kp = 10;
  geometry_msgs::Twist twist;
  twist.linear.x = 0.5;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = angle*kp;
  publisher_twist_.publish(twist);

  // Mark the target position in rviz
  float x, y, z;
  x = min_dis * cos(angle);
  y = min_dis * sin(angle);
  z = 0;
  ROS_INFO("Pillar Position is: X: %f, Y: %f, Z: %f", x, y, z);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.lifetime;
  publisher_rvizMarker_.publish(marker);
}  

void SmbHighlevelController::topic_pointcloud_Callback(const sensor_msgs::PointCloud2 data)
{
  int size = data.height * data.width;
  ROS_INFO("Pointcloud size: %i", size);
}
} /* namespace */
