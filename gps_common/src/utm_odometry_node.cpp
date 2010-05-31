/*
 * Translates gps_common/GPS{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <gps_common/GPSStatus.h>
#include <gps_common/GPSFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>

using namespace gps_common;

static ros::Publisher odom_pub;

void callback(const GPSStatusConstPtr& status, const GPSFixConstPtr& fix) {
  if (status->status == GPSStatus::STATUS_NO_FIX) {
    ROS_INFO("No fix.");
    return;
  }

  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

  if (odom_pub) {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;
    odom.pose.pose.position.x = easting;
    odom.pose.pose.position.y = northing;
    odom.pose.pose.position.z = 0.0;

    odom_pub.publish(odom);
  }
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle node;

  message_filters::Subscriber<GPSStatus> status_sub(node, "status", 1);
  message_filters::Subscriber<GPSFix> fix_sub(node, "fix", 1);
  message_filters::TimeSynchronizer<GPSStatus, GPSFix> sync(status_sub, fix_sub, 10);

  odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);

  sync.registerCallback(
    boost::bind(&callback, _1, _2)
  );

  ros::spin();
}
