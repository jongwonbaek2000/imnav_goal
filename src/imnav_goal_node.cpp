#include <imnav_goal/imnav_goal.h>

ImnavGoal::ImnavGoal() : origin_received_(false), new_goal_received_(false) {
    origin_sub_ = nh_.subscribe("/origin_gps", 1, &ImnavGoal::originCallback, this);
    goal_sub_ = nh_.subscribe("/gps_goal_fix", 1, &ImnavGoal::goalCallback, this);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
}

void ImnavGoal::originCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    if (!origin_received_) {
        origin_gps_ = *msg;
        origin_received_ = true;
        ROS_INFO("Origin GPS received: Lat=%f, Lon=%f", origin_gps_.latitude, origin_gps_.longitude);
    }
}

void ImnavGoal::goalCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    if (!origin_received_) {
        ROS_WARN("Origin GPS not received yet. Ignoring goal.");
        return;
    }

    last_goal_ = *msg;
    new_goal_received_ = true;
    ROS_INFO("Goal GPS received: Lat=%f, Lon=%f", last_goal_.latitude, last_goal_.longitude);
    processAndPublishGoal();
}

void ImnavGoal::processAndPublishGoal() {
    if (!new_goal_received_) {
        return;
    }

    auto [origin_x, origin_y, origin_zone] = gpsToUTM(origin_gps_.latitude, origin_gps_.longitude);
    auto [goal_x, goal_y, goal_zone] = gpsToUTM(last_goal_.latitude, last_goal_.longitude);

    if (origin_zone != goal_zone) {
        ROS_WARN("Origin and goal are in different UTM zones. This may cause inaccuracies.");
    }
    ROS_INFO("Origin UTM: x=%f, y=%f", origin_x, origin_y);
    ROS_INFO("Goal UTM: x=%f, y=%f", goal_x, goal_y);
    double x_star = goal_x - origin_x;
    double y_star = goal_y - origin_y;
    
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position.x = x_star;
    goal_msg.pose.position.y = y_star;
    goal_msg.pose.orientation.w = 1.0;

    goal_pub_.publish(goal_msg);
    ROS_INFO("Published new goal: x=%f, y=%f", x_star, y_star);

    new_goal_received_ = false;
}

std::tuple<double, double, int> ImnavGoal::gpsToUTM(double lat, double lon) {
    int zone;
    bool northp;
    double x, y;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
    return std::make_tuple(x, y, zone);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imnav_goal_node");
    ImnavGoal imnav_goal;
    ros::spin();
    return 0;
}