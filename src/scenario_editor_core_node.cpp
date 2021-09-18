#include <ros/ros.h>
#include "scenario_editor_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scenario_editor_node");
    ScenarioEditorCore scenario_editor_core;
    ros::spin();
    return 0;
}