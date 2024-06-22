#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

std::vector<std::tuple<double, double, double>> readGoalsFromCSV(const std::string &csv_file) {
    std::ifstream file(csv_file);
    std::string line;
    std::vector<std::tuple<double, double, double>> goals;

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string token;
        double x, y, theta;

        std::getline(ss, token, ',');
        x = std::stod(token);
        std::getline(ss, token, ',');
        y = std::stod(token);
        std::getline(ss, token, ',');
        theta = std::stod(token);

        goals.emplace_back(x, y, theta);
    }

    return goals;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_publisher");
    ros::NodeHandle nh("~");

    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);
    ros::Rate sleep_rate(0.05);
    std::string csv_file;
    if (!nh.getParam("csv_file", csv_file)) {
        ROS_ERROR("Failed to get param 'csv_file'");
        return 1;
    }

    std::vector<std::tuple<double, double, double>> goals = readGoalsFromCSV(csv_file);
    ROS_INFO("%li have been load from %s", goals.size(), csv_file.c_str());
    int current_goal = 0;
    while (ros::ok() && current_goal < goals.size())
    {
        
        geometry_msgs::PoseStamped goal_msg;
        double x = std::get<0>(goals.at(current_goal));
        double y = std::get<1>(goals.at(current_goal));
        double z = std::get<2>(goals.at(current_goal));
        goal_msg.header.frame_id = "map";
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.pose.position.x = x;
        goal_msg.pose.position.y = y;
        goal_msg.pose.orientation.z = sin(z / 2.0);
        goal_msg.pose.orientation.w = cos(z / 2.0);

        goal_pub.publish(goal_msg);
        ROS_INFO("Published waypoint at: %f %f %f", x, y, z);
        sleep_rate.sleep();
        ros::spinOnce();
        current_goal++;
    }
    ROS_INFO("All goals have been published.");

    return 0;
}