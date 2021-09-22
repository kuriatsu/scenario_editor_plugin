#ifndef SCENARIO_EDITOR_H
#define SCENARIO_EDITOR_H

#include <QWidget>
#include <QFileDialog>
#include <fstream>
#include <sstream>
#include "nlohmann/json.hpp"
#include <QDebug>
#include <QListWidgetItem>
#include <iomanip>
#include "scenario_editor_core.h"
#include <time.h>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <rviz/panel.h>
#endif

using json = nlohmann::json;

namespace Ui {
class ScenarioEditorPlugin;
}

namespace scenario_editor_plugin
{

class ScenarioEditorPlugin : public rviz::Panel, ScenarioEditorCore
{
    Q_OBJECT

public:
    explicit ScenarioEditorPlugin(QWidget *parent = 0);
    ~ScenarioEditorPlugin();

private Q_SLOTS:
    void on_waypoint_button_pressed();

    void on_scenario_button_pressed();

    void on_id_score_button_pressed();

    void on_start_button_pressed();

    void on_candidate_list_itemDoubleClicked(QListWidgetItem *item);

    void on_included_list_itemDoubleClicked(QListWidgetItem *item);

    void on_add_scenario_botton_pressed();

    void on_remove_scenario_button_pressed();

    void on_tabWidget_tabBarClicked(int index);

    void on_save_button_pressed();

    void on_candidate_list_itemClicked(QListWidgetItem *item);

    void on_included_list_itemClicked(QListWidgetItem *item);

    void on_layer_size_value_editingFinished();

    void on_layer_value_valueChanged(int arg1);

    void on_speed_value_editingFinished();

    void on_highlight_button_clicked(bool checked);

private:
    Ui::ScenarioEditorPlugin *ui;
    QString m_waypoint_file;
    QString m_scenario_file;
    QString m_id_score_file;
    std::vector<std::vector<std::string>> m_id_score;
    std::vector<std::vector<std::string>> m_waypoint;
    json m_scenario;
    int m_selected_scenario_id;
    int m_layer = 0;
    int m_layer_size = 300;
    bool is_add_scenario_mode = false;
    clock_t m_last_fb_time = 0.0;

    std::vector<std::vector<float>> m_color_list{
        {1.0, 0.0, 0.0, 1.0},
        {1.0, 0.7, 0.0, 1.0},
        {1.0, 1.0, 0.0, 1.0},
        {0.7, 1.0, 0.0, 1.0},
        {0.0, 1.0, 0.0, 1.0},
        {0.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 1.0, 1.0},
        {1.0, 0.0, 1.0, 1.0},
        {0.5, 0.0, 0.5, 1.0},
    };

    void read_csv(const std::string filename, std::vector<std::vector<std::string>> &out_list);
    void read_json(const std::string filename, json &out_list);
    void updateScenarioPanel(const int selected_node);
    void scenarioEditMode();
    void addScenarioMode();
    void highlightSelectedScenario();
    void updateErrorList(const int &id);
    int insertNewScenario(const std::string &start, const std::string &end);
    void buttonCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void scenarioCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
};
}
#endif // SCENARIO_EDITOR_H
