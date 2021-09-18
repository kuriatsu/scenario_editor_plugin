#include <ros/ros.h>
#include "scenario_editor_core.h"
#include <fstream>
#include <sstream>


void read_csv(const std::string filename, std::vector<std::vector<std::string>>& out_list)
{    
    std::ifstream ifs(filename);
    std::string line;
    std::getline(ifs, line);
    while(std::getline(ifs, line))
    {
        std::istringstream stream(line);
        std::string field;
        std::vector<std::string> result;
        while(std::getline(stream, field, ','))
        {
            result.emplace_back(field);
        }
        out_list.emplace_back(result);
    }
}


ScenarioEditorCore::ScenarioEditorCore()
{
    ros::NodeHandle n;
    int_marker_server.reset(new interactive_markers::InteractiveMarkerServer("scenario_editor_node"));
    pub_lines = n.advertise<visualization_msgs::MarkerArray>("/lines", 5, true);
    pub_points = n.advertise<visualization_msgs::MarkerArray>("/points", 5, true);
}

ScenarioEditorCore::~ScenarioEditorCore()
{
    clearLines();
    int_marker_server.reset();
}

void ScenarioEditorCore::showLines(const std::vector<std::vector<std::vector<std::string>>> &lines, const std::vector<std::vector<float>> &colors)
{
    clearLines();
    int i = 0;
    visualization_msgs::MarkerArray array;
    for (const auto &line : lines)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "line";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.0);
        marker.frame_locked = false;
        marker.scale.x = 0.2;
        marker.scale.y = 0.0;
        marker.scale.z = 0.0;
        if (lines.size() == colors.size())
        {
            marker.color.r = colors[i][0];
            marker.color.g = colors[i][1];
            marker.color.b = colors[i][2];
            marker.color.a = colors[i][3];
        }
        else
        {
            marker.color.r = colors[0][0];
            marker.color.g = colors[0][1];
            marker.color.b = colors[0][2];
            marker.color.a = colors[0][3];
        }

        for (const auto &waypoint : line)
        {
            geometry_msgs::Point point;
            point.x = std::stof(waypoint[0]);
            point.y = std::stof(waypoint[1]);
            point.z = std::stof(waypoint[2]) - 1.0;
            marker.points.emplace_back(point);
        }
        array.markers.push_back(marker);
        i++;
    }
    pub_lines.publish(array);
}


void ScenarioEditorCore::showPoints(const std::vector<std::vector<std::string>> &points, const std::vector<float> &color)
{
    int point_id = 0;
    visualization_msgs::MarkerArray array;
    for (const auto &point : points)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "points";
        marker.id = point_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.0);
        marker.frame_locked = false;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = color[3];
        marker.pose.position.x = std::stof(point[0]);
        marker.pose.position.y = std::stof(point[1]);
        marker.pose.position.z = std::stof(point[2]);
        array.markers.push_back(marker);
        point_id++;
    }
    pub_points.publish(array);
}

void ScenarioEditorCore::showArrows(const std::vector<std::vector<std::string>> &points, const std::vector<float> &color)
{
    int point_id = 0;
    visualization_msgs::MarkerArray array;
    for (const auto &point : points)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "arrows";
        marker.id = point_id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.0);
        marker.frame_locked = false;
        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = color[3];
        marker.pose.position.x = std::stof(point[0]);
        marker.pose.position.y = std::stof(point[1]);
        marker.pose.position.z = std::stof(point[2]);
        marker.pose.orientation = yawToQuat(std::stof(point[3]));
        array.markers.push_back(marker);
        point_id++;
    }
    pub_points.publish(array);
}

void ScenarioEditorCore::makeIntScenarios(const std::vector<std::string> &id_list, const std::vector<std::vector<std::string>> &waypoints, const std::vector<std::vector<float>> &colors)
{
    int i = 0;
    for (const auto &waypoint : waypoints)
    {
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = "map";
        int_marker.name = id_list[i];
        int_marker.scale = 1.0;
        int_marker.pose.position.x = std::stof(waypoint[0]);
        int_marker.pose.position.y = std::stof(waypoint[1]);
        int_marker.pose.position.z = std::stof(waypoint[2]);
        int_marker.pose.orientation.x = 0.0;
        int_marker.pose.orientation.y = 1.0;
        int_marker.pose.orientation.z = 0.0;
        int_marker.pose.orientation.w = 1.0;

        visualization_msgs::Marker marker;
        marker.ns = "scenario";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        if (colors.size() == waypoints.size())
        {
            marker.color.r = colors[i][0];
            marker.color.g = colors[i][1];
            marker.color.b = colors[i][2];
            marker.color.a = colors[i][3];
        }
        else
        {
            marker.color.r = colors[0][0];
            marker.color.g = colors[0][1];
            marker.color.b = colors[0][2];
            marker.color.a = colors[0][3];
        }

        visualization_msgs::InteractiveMarkerControl move_control;
        move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
        move_control.name = "move_xy";
        int_marker.controls.emplace_back(move_control);

        visualization_msgs::InteractiveMarkerControl button_control;
        button_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
        button_control.always_visible = true;
        button_control.name = "button";
        button_control.markers.emplace_back(marker);
        int_marker.controls.emplace_back(button_control);

        int_marker_server->insert(int_marker);
        int_marker_server->setCallback(int_marker.name, boost::bind(&ScenarioEditorCore::scenarioCb, this, _1));

        i++;
    }
    int_marker_server->applyChanges();
}

void ScenarioEditorCore::makeButtonWaypoints(const std::vector<std::string> &id_list, const std::vector<std::vector<std::string>> &waypoints, const std::vector<std::vector<float>> &colors)
{
    int i = 0;
    for (const auto &waypoint : waypoints)
    {
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = "map";
        int_marker.name = id_list[i];
        int_marker.scale = 1.0;
        int_marker.pose.position.x = std::stof(waypoint[0]);
        int_marker.pose.position.y = std::stof(waypoint[1]);
        int_marker.pose.position.z = std::stof(waypoint[2]);
        int_marker.pose.orientation.x = 0.0;
        int_marker.pose.orientation.y = 0.0;
        int_marker.pose.orientation.z = 1.0;
        int_marker.pose.orientation.w = 1.0;

        visualization_msgs::InteractiveMarkerControl control;
        control.always_visible = true;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
        control.name = "button";

        visualization_msgs::Marker marker;
        marker.ns = "waypoint";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.1;
        if (colors.size() == waypoints.size())
        {
            marker.color.r = colors[i][0];
            marker.color.g = colors[i][1];
            marker.color.b = colors[i][2];
            marker.color.a = colors[i][3];
        }
        else
        {
            marker.color.r = colors[0][0];
            marker.color.g = colors[0][1];
            marker.color.b = colors[0][2];
            marker.color.a = colors[0][3];
        }

        control.markers.emplace_back(marker);
        int_marker.controls.emplace_back(control);

        int_marker_server->insert(int_marker);
        int_marker_server->setCallback(int_marker.name, boost::bind(&ScenarioEditorCore::buttonCb, this, _1));
        i++;
    }
    int_marker_server->applyChanges();
}

geometry_msgs::Quaternion ScenarioEditorCore::yawToQuat(const double yaw)
{
    tf::Quaternion tf_quat = tf::createQuaternionFromRPY(0.0, 0.0, yaw);
    geometry_msgs::Quaternion quat;
    quaternionTFToMsg(tf_quat, quat);
    return quat;
}

void ScenarioEditorCore::clearLines()
{
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker.header.frame_id = "map";
    array.markers.push_back(marker);
    pub_lines.publish(array);
}


void ScenarioEditorCore::clearPoints()
{
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker.header.frame_id = "map";
    array.markers.push_back(marker);
    pub_points.publish(array);
}


void ScenarioEditorCore::clearIntMarkers(const std::vector<std::string> &names)
{
    for (const auto &name : names)
    {
        int_marker_server->erase(name);
    }
    int_marker_server->applyChanges();
}

void ScenarioEditorCore::clearAllIntMarkers()
{
    int_marker_server->clear();
    int_marker_server->applyChanges();
}

void ScenarioEditorCore::buttonCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::cout << feedback->marker_name << std::endl;
}

void ScenarioEditorCore::scenarioCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    {
        std::cout << feedback->marker_name << std::endl;
        int waypoint_index = getClosestWaypoint(m_waypoint, feedback->pose, 0, m_waypoint.size());
        geometry_msgs::Pose pose;
        pose.position.x = std::stof(m_waypoint[waypoint_index][0]);
        pose.position.y = std::stof(m_waypoint[waypoint_index][1]);
        pose.position.z = std::stof(m_waypoint[waypoint_index][2])+0.5;
        pose.orientation.x = 0.0;
        pose.orientation.y = 1/std::sqrt(2);
        pose.orientation.z = 0.0;
        pose.orientation.w = 1/std::sqrt(2);
        int_marker_server->setPose(feedback->marker_name, pose);
        int_marker_server->applyChanges();
    }
}


int ScenarioEditorCore::getClosestWaypoint(const std::vector<std::vector<std::string>> &waypoints, const geometry_msgs::Pose &pose, const int start, const int end)
{
    float min_dist = 1000000.0;
    int closest_waypoint = 0;
    for (size_t i = start; i < end; ++i)
    {
        if (i > waypoints.size()-1) return 0;
        float dist = std::sqrt(std::pow(std::stof(waypoints[i][0])-pose.position.x, 2) + std::pow(std::stof(waypoints[i][1])-pose.position.y, 2));
        if (dist < min_dist)
        {
            min_dist = dist;
            closest_waypoint = i;
        }
    }
    return closest_waypoint;
}
