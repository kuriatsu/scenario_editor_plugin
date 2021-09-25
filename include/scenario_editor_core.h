/**
* @file scenario_editor_core.h
* @brief Rviz side handler library header file for scenario_editor_plugin package. Show/Remove markers.
* @author Atsushi Kuribayashi (kuriatsubayashi712@gmail.com)
* @date 2021/9
*
* @details
* @note
*/

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

/**
 * @brief The ScenarioEditorCore class
 */
class ScenarioEditorCore
{
private:
    /**
     * @brief pub_lines /lines message publisher
     */
    ros::Publisher pub_lines;
    /**
     * @brief pub_points /points message publisher
     */
    ros::Publisher pub_points;

public:
    /**
     * @brief ScenarioEditorCore Rviz side handler. Show/Remove markers.
     */
    ScenarioEditorCore();
    /**
     * @brief ~ScenarioEditorCore Remove markers, reset interactive marker server.
     */
    ~ScenarioEditorCore();
    /**
     * @brief int_marker_server interactive marker server
     */
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> int_marker_server;


private:
    /**
     * @brief yawToQuat Calcurate Quaternion from yaw
     * @param yaw [radian]
     * @return geometry_msgs::Quaternion
     */
    geometry_msgs::Quaternion yawToQuat(const double yaw);

protected:
    /**
     * @brief m_waypoint waypoint list (for discrete run)
     */
    std::vector<std::vector<std::string>> m_waypoint;

    /**
     * @brief buttonCb Callback function for interactive marker. Just output id.
     * @param feedback Callback information
     */
    virtual void buttonCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    /**
     * @brief scenarioCb Callback function for interactive marker. When moved, find closest waypoint and realign on it.
     * @param feedback Callback information
     */
    virtual void scenarioCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    /**
     * @brief makeButtonWaypoints Create button markers from waypoints
     * @param id_list ID for each waypoint
     * @param waypoints Button position list [x, y, z, ...]
     * @param colors Button color list
     */
    void makeButtonWaypoints(const std::vector<std::string> &id_list, const std::vector<std::vector<std::string>> &waypoints, const std::vector<std::vector<float>> &colors);
    /**
     * @brief makeIntScenarios Create button and move-plane-xy markers from waypoints
     * @param id_list ID for each waypoint
     * @param waypoints Button position list [x, y, z, yaw, ...]
     * @param colors Button color list
     */
    void makeIntScenarios(const std::vector<std::string> &id_list, const std::vector<std::vector<std::string>> &waypoints, const std::vector<std::vector<float>> &colors);
    /**
     * @brief showLines Make line marker from segment list consists from waypoints. Publish to /lines
     * @param lines Segment list consists from waypoints
     * @param colors Line colors
     * @param ns Marker namespace (avoid conflict from id duplication)
     */
    void showLines(const std::vector<std::vector<std::vector<std::string>>> &lines, const std::vector<std::vector<float>> &colors, const std::string ns = "");
    /**
     * @brief showPoints Make point marker from points. Publish to /points
     * @param points Waypoint list [x, y, z]
     * @param color Point color
     * @param ns Marker namespace  (avoid conflict from id duplication)
     */
    void showPoints(const std::vector<std::vector<std::string>> &points, const std::vector<float> &color, const std::string ns = "");
    /**
     * @brief showArrows Make arrow marker from points. Publish to /points
     * @param points points Waypoint list [x, y, z, yaw, ...]
     * @param color Point color
     * @param ns Marker namespace (avoid conflict from id duplication)
     */
    void showArrows(const std::vector<std::vector<std::string>> &points, const std::vector<float> &color, const std::string ns = "");
    /**
     * @brief clearLines Remove all line marker published to /lines
     */
    void clearLines();
    /**
     * @brief clearPoints Remove all point marker published to /points
     */
    void clearPoints();
    /**
     * @brief clearAllIntMarkers Remove all interactive markers
     */
    void clearAllIntMarkers();
    /**
     * @brief clearIntMarkers Remove specific interactive marker
     * @param names
     */
    void clearIntMarkers(const std::vector<std::string> &names);
    /**
     * @brief getClosestWaypoint Find closest waypoint (start->end) from pose in waypoints
     * @param waypoints Search target
     * @param pose Search target
     * @param start Search range
     * @param end Search range
     * @return int Result
     */
    int getClosestWaypoint(const std::vector<std::vector<std::string>> &waypoints, const geometry_msgs::Pose &pose, const int start, const int end);

};
