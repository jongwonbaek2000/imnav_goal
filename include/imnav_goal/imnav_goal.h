#ifndef IMNAV_GOAL_H
#define IMNAV_GOAL_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <GeographicLib/UTMUPS.hpp>

class ImnavGoal {
public:
    ImnavGoal();

private:
    ros::NodeHandle nh_;
    ros::Subscriber origin_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher goal_pub_;

    sensor_msgs::NavSatFix origin_gps_;
    bool origin_received_;
    sensor_msgs::NavSatFix last_goal_;
    bool new_goal_received_;

    void originCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void goalCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    std::tuple<double, double, int> gpsToUTM(double lat, double lon);
    void processAndPublishGoal();
};

#endif // IMNAV_GOAL_H