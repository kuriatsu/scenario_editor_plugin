#pragma once

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_broadcaster.h>

class ScenarioEditorCore
{
private:
    ros::Publisher pub_lines;
    ros::Publisher pub_points;

public:
    ScenarioEditorCore();
    ~ScenarioEditorCore();
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> int_marker_server;


private:
    geometry_msgs::Quaternion yawToQuat(const double yaw);

protected:
    std::vector<std::vector<std::string>> m_waypoint;
    virtual void buttonCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    virtual void scenarioCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void makeButtonWaypoints(const std::vector<std::string> &id_list, const std::vector<std::vector<std::string>> &waypoints, const std::vector<std::vector<float>> &colors);
    void makeIntScenarios(const std::vector<std::string> &id_list, const std::vector<std::vector<std::string>> &waypoints, const std::vector<std::vector<float>> &colors);
    void showLines(const std::vector<std::vector<std::vector<std::string>>> &lines, const std::vector<std::vector<float>> &colors);
    void showPoints(const std::vector<std::vector<std::string>> &points, const std::vector<float> &color);
    void showArrows(const std::vector<std::vector<std::string>> &points, const std::vector<float> &color);
    void clearLines();
    void clearPoints();
    void clearAllIntMarkers();
    void clearIntMarkers(const std::vector<std::string> &names);
    int getClosestWaypoint(const std::vector<std::vector<std::string>> &waypoints, const geometry_msgs::Pose &pose, const int start, const int end);

};
