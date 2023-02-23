#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

namespace smb_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class SmbHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	SmbHighlevelController(ros::NodeHandle& nodeHandle);
	

	/*!
	 * Destructor.
	 */
	virtual ~SmbHighlevelController();

private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber subscriber_laser_;
	ros::Subscriber subscriber_pointcloud_;

	void topic_laser_Callback(const sensor_msgs::LaserScan data);
	void topic_pointcloud_Callback(const sensor_msgs::PointCloud2 data);
};

} /* namespace */
