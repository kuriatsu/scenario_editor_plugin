/**
* @file
* @brief
* @author
* @date
*
* @details
* @note
*/

#include "scenario_editor_plugin.h"
#include "ui_scenarioeditorplugin.h"

namespace scenario_editor_plugin
{

ScenarioEditorPlugin::ScenarioEditorPlugin(QWidget *parent) :
    // QWidget(parent),
    rviz::Panel(parent),
    ui(new Ui::ScenarioEditorPlugin)
{
    ui->setupUi(this);
}

ScenarioEditorPlugin::~ScenarioEditorPlugin()
{
    delete ui;
}

void ScenarioEditorPlugin::on_waypoint_button_pressed()
{
    m_waypoint_file = QFileDialog::getOpenFileName(this, tr("open file"), "~/", tr("csv file (*csv)"));
    ui->waypoint_text->setText(m_waypoint_file);
    try
    {
        read_csv(m_waypoint_file.toStdString(), m_waypoint);
        ui->start_button->setDisabled(false);
    }
    catch(char *error)
    {
        ui->waypoint_text->setText(QString::fromStdString(std::string(error)));
        ui->start_button->setDisabled(true);
    }

}

void ScenarioEditorPlugin::on_id_score_button_pressed()
{
    m_id_score_file = QFileDialog::getOpenFileName(this, tr("open file"), "~/", tr("csv file (*csv)"));
    ui->id_score_text->setText(m_id_score_file);
    try
    {
        read_csv(m_id_score_file.toStdString(), m_id_score);
        ui->start_button->setDisabled(false);
    }
    catch (char *error)
    {
        ui->id_score_text->setText(QString::fromStdString(std::string(error)));
        ui->start_button->setDisabled(true);
    }

}

void ScenarioEditorPlugin::on_scenario_button_pressed()
{
    m_scenario_file = QFileDialog::getOpenFileName(this, tr("open file"), "~/", tr("json file (*json)"));
    ui->scenario_text->setText(m_scenario_file);
    try
    {
        read_json(m_scenario_file.toStdString(), m_scenario);
        ui->start_button->setDisabled(false);
    }
    catch (char *error)
    {
        ui->scenario_text->setText(QString::fromStdString(std::string(error)));
        ui->start_button->setDisabled(true);
    }
}

void ScenarioEditorPlugin::on_start_button_pressed()
{
    if(ui->start_button->text() == "OK")
    {
        ui->scenario_button->setDisabled(true);
        ui->scenario_text->setDisabled(true);
        ui->waypoint_button->setDisabled(true);
        ui->waypoint_text->setDisabled(true);
        ui->id_score_button->setDisabled(true);
        ui->id_score_text->setDisabled(true);
        ui->start_button->setText(tr("Reload"));
        ui->save_button->setEnabled(true);
    }
    else
    {
        ui->scenario_button->setDisabled(false);
        ui->scenario_text->setDisabled(false);
        ui->waypoint_button->setDisabled(false);
        ui->waypoint_text->setDisabled(false);
        ui->id_score_button->setDisabled(false);
        ui->id_score_text->setDisabled(false);
        ui->start_button->setText(tr("OK"));
        ui->save_button->setDisabled(true);
    }
}

void ScenarioEditorPlugin::on_save_button_pressed()
{
    QString file = QFileDialog::getSaveFileName(this, tr("save file"), "~/", tr("json file (*json)"));
    std::ofstream o(file.toStdString());
    o << std::setw(4) << m_scenario << std::endl;
}

void ScenarioEditorPlugin::on_add_scenario_botton_pressed()
{
    if (ui->add_scenario_botton->text().compare("Add Scenario", Qt::CaseSensitive) == 0)
    {
        ui->add_scenario_botton->setText(tr("Cancel"));
        ui->scenario_start_label->setStyleSheet("background-color : rgb(100, 200, 0);");
        ui->scenario_start_text->setStyleSheet("background-color : rgb(100, 200, 0);");
        ui->current_scenario_text->setText(tr("---"));
        ui->scenario_start_text->setText(tr("---"));
        ui->scenario_end_text->setText(tr("---"));
        m_is_add_scenario_mode = true;
        makeWaypointIntMarker();
    }
    else
    {
        ui->add_scenario_botton->setText(tr("Add Scenario"));
        ui->scenario_start_label->setStyleSheet("background-color : transparent;");
        ui->scenario_start_text->setStyleSheet("background-color : transparent;");
        ui->scenario_start_label->setStyleSheet("background-color : transparent;");
        ui->scenario_start_text->setStyleSheet("background-color : transparent;");
        ui->current_scenario_text->setText(QString::number(m_selected_scenario_id));
        ui->scenario_start_text->setText(QString::fromStdString(m_scenario[m_selected_scenario_id]["start_id"]));
        ui->scenario_end_text->setText(QString::fromStdString(m_scenario[m_selected_scenario_id]["end_id"]));
        m_is_add_scenario_mode = false;
        makeScenarioEditIntMarker();
    }

}

void ScenarioEditorPlugin::on_remove_scenario_button_pressed()
{
    m_scenario.erase(m_scenario.begin()+m_selected_scenario_id);
    if (m_selected_scenario_id == m_scenario.size()-1)
        m_selected_scenario_id -= 1;
    ui->current_scenario_text->setText(QString::number(m_selected_scenario_id));
    ui->scenario_start_text->setText(QString::fromStdString(m_scenario[m_selected_scenario_id]["start_id"]));
    ui->scenario_end_text->setText(QString::fromStdString(m_scenario[m_selected_scenario_id]["end_id"]));
    updatePanel(m_selected_scenario_id);
    updateHighlight();
    makeScenarioEditIntMarker();
}

void ScenarioEditorPlugin::on_tabWidget_tabBarClicked(int index)
{
    switch (index)
    {
        case 0:
            break;
        case 1:
            if (m_waypoint.size() > 0)
            {
                makeScenarioEditIntMarker();
                updatePanel(m_selected_scenario_id);
            }
            break;
    }
}

void ScenarioEditorPlugin::on_layer_size_value_editingFinished()
{
    m_layer_size = ui->layer_size_value->value();
    ui->layer_value->setMaximum(m_waypoint.size()/m_layer_size);
    if (m_is_add_scenario_mode) makeWaypointIntMarker();
    else makeScenarioEditIntMarker();
}

void ScenarioEditorPlugin::on_layer_value_valueChanged(int arg1)
{
    m_layer = arg1;
    if (m_is_add_scenario_mode) makeWaypointIntMarker();
    else makeScenarioEditIntMarker();
}


void ScenarioEditorPlugin::on_candidate_list_itemDoubleClicked(QListWidgetItem *item)
{
    ui->candidate_list->takeItem(ui->candidate_list->row(item));
    ui->included_list->addItem(item->text());
    ui->candidate_list->sortItems(Qt::AscendingOrder);
    m_scenario[m_selected_scenario_id]["errors"].emplace_back(item->text().toStdString());
    delete item;
    updateHighlight();
}

void ScenarioEditorPlugin::on_included_list_itemDoubleClicked(QListWidgetItem *item)
{
    ui->included_list->takeItem(ui->included_list->row(item));
    ui->candidate_list->addItem(item->text());
    ui->candidate_list->sortItems(Qt::AscendingOrder);

    auto &errors = m_scenario[m_selected_scenario_id]["errors"];
    for (auto itr = errors.begin(); itr != errors.end(); ++itr)
    {
        if (*itr == item->text().toStdString())
        {
            errors.erase(itr);
            delete item;
            return;
        }
    }

    updateHighlight();
}

void ScenarioEditorPlugin::on_candidate_list_itemClicked(QListWidgetItem *item)
{
    for (std::vector<std::string>& id_score : m_id_score)
    {
        if (item->text().compare(QString::fromStdString(id_score[0]), Qt::CaseSensitive) == 0)
        {
            ui->label1_text->setText(QString::fromLocal8Bit(id_score[1].c_str()));
            ui->label2_text->setText(QString::fromLocal8Bit(id_score[2].c_str()));
            ui->label3_text->setText(QString::fromLocal8Bit(id_score[3].c_str()));
            ui->score_text->setText(QString::fromLocal8Bit(id_score[4].c_str()));
            ui->message_text->setText(QString::fromLocal8Bit(id_score[5].c_str()));
            ui->description_text->setText(QString::fromLocal8Bit(id_score[6].c_str()));
            break;
        }
    }
}

void ScenarioEditorPlugin::on_included_list_itemClicked(QListWidgetItem *item)
{
    for (std::vector<std::string>& id_score : m_id_score)
    {
        if (item->text().compare(QString::fromStdString(id_score[0]), Qt::CaseSensitive) == 0)
        {
            ui->label1_text->setText(QString::fromLocal8Bit(id_score[1].c_str()));
            ui->label2_text->setText(QString::fromLocal8Bit(id_score[2].c_str()));
            ui->label3_text->setText(QString::fromLocal8Bit(id_score[3].c_str()));
            ui->score_text->setText(QString::fromLocal8Bit(id_score[4].c_str()));
            ui->message_text->setText(QString::fromLocal8Bit(id_score[5].c_str()));
            ui->description_text->setText(QString::fromLocal8Bit(id_score[6].c_str()));
            break;
        }
    }
}


void ScenarioEditorPlugin::on_speed_value_editingFinished()
{
    m_scenario[m_selected_scenario_id]["speed_limit"] = std::to_string(ui->speed_value->value());
}

void ScenarioEditorPlugin::on_highlight_button_clicked(bool checked)
{
    if (checked)
    {
        if (ui->candidate_list->selectedItems().size() + ui->included_list->selectedItems().size() != 1)
        {
            m_highlight_error.clear();
            ui->highlight_button->setChecked(false);
        }
        else if (!ui->candidate_list->selectedItems().empty())
            m_highlight_error = ui->candidate_list->selectedItems()[0]->text();
        else if (!ui->included_list->selectedItems().empty())
            m_highlight_error = ui->included_list->selectedItems().at(0)->text();
    }
    else
    {
        m_highlight_error.clear();
    }
    updateHighlight();
}

void ScenarioEditorPlugin::read_csv(const std::string filename, std::vector<std::vector<std::string>> &out_list)
{
    out_list.clear();

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

void ScenarioEditorPlugin::read_json(const std::string filename, json &out_list)
{
    out_list.clear();
    std::ifstream ifs(filename);
    ifs >> out_list;
}

void ScenarioEditorPlugin::updatePanel(const int &id)
{
    if (m_scenario.size() == 0 || m_waypoint.empty() || m_id_score.empty()) return;

    ui->layer_value->setMaximum(m_waypoint.size()/m_layer_size);
    ui->current_scenario_text->setText(QString::number(id));
    ui->scenario_start_text->setText(QString::fromStdString(m_scenario[id]["start_id"]));
    ui->scenario_end_text->setText(QString::fromStdString(m_scenario[id]["end_id"]));
    ui->speed_value->setValue(std::stoi((std::string)m_scenario[id]["speed_limit"]));

    updateErrorList(id);
    return;
}

void ScenarioEditorPlugin::updateErrorList(const int &id)
{
    ui->included_list->clear();
    ui->candidate_list->clear();
    // add to candidate/include list
    for (std::vector<std::string> &id_score : m_id_score)
    {
        bool included_flag = false;
        for (const std::string &error : m_scenario[id]["errors"])
        {
            if (error ==  id_score[0])
            {
                included_flag = true;
                break;
            }
        }
        // add to include list
        if (included_flag)
        {
            ui->included_list->addItem(QString::fromStdString(id_score[0]));
        }
        // add to candidate list
        else
        {
            ui->candidate_list->addItem(QString::fromStdString(id_score[0]));
        }
    }
    ui->candidate_list->sortItems(Qt::AscendingOrder);
}

void ScenarioEditorPlugin::updateHighlight()
{
    clearLines();
    highlightScenarioSelected(m_selected_scenario_id);
    highlightScenarioError(m_highlight_error);
}

void ScenarioEditorPlugin::highlightScenarioSelected(const int &id)
{
    // check start and end point order
    int start_id = std::stoi((std::string)m_scenario[id]["start_id"]);
    int end_id = std::stoi((std::string)m_scenario[id]["end_id"]);

    if (start_id >= end_id)
    {
        ui->scenario_start_label->setStyleSheet("background-color : rgb(200, 0, 0);");
        ui->scenario_start_text->setStyleSheet("background-color : rgb(200, 0, 0);");
        ui->scenario_end_label->setStyleSheet("background-color : rgb(200, 0, 0);");
        ui->scenario_end_text->setStyleSheet("background-color : rgb(200, 0, 0);");
        return;
    }
    else
    {
        ui->scenario_start_label->setStyleSheet("background-color : transparent;");
        ui->scenario_start_text->setStyleSheet("background-color : transparent;");
        ui->scenario_end_label->setStyleSheet("background-color : transparent;");
        ui->scenario_end_text->setStyleSheet("background-color : transparent;");
    }

    std::vector<std::vector<std::string>> waypoints;
    std::vector<std::vector<std::vector<std::string>>> lines;
    std::vector<std::vector<float>> colors = {{1.0, 0.5, 0.5, 1.0}};
    for (int i = start_id; i <= end_id; i++)
    {
        if (i > m_waypoint.size()-1) break;
        std::vector<std::string> buf_waypoint(m_waypoint[i].size());
        std::copy(m_waypoint[i].begin(), m_waypoint[i].end(), buf_waypoint.begin());
        buf_waypoint[2] = std::to_string(std::stof(buf_waypoint[2]) + 1.0);
        waypoints.emplace_back(buf_waypoint);
    }
    lines.emplace_back(waypoints);
    showLines(lines, colors, "selected");
}

void ScenarioEditorPlugin::highlightScenarioError(const QString &highlight_error)
{
    if (highlight_error.isEmpty()) return;

    std::vector<std::vector<std::vector<std::string>>> lines;
    std::vector<std::vector<float>> colors = {{1.0, 1.0, 0.0, 1.0}};
    for (const auto &scenario : m_scenario)
    {
        bool is_highlight = false;
        std::vector<std::vector<std::string>> waypoints;

        if ((m_layer+1)*m_layer_size <= std::stoi((std::string)scenario["start_id"]) || std::stoi((std::string)scenario["end_id"]) < m_layer*m_layer_size)
            continue;

        for (const std::string &error : scenario["errors"])
        {
            if (highlight_error.compare(QString::fromStdString(error), Qt::CaseSensitive) != 0)
                continue;
            is_highlight = true;
        }

        if (!is_highlight) continue;
        for (int i = std::stoi((std::string)scenario["start_id"]); i <= std::stoi((std::string)scenario["end_id"]); i++)
        {
            if (i > m_waypoint.size()-1) break;
            std::vector<std::string> buf_waypoint(m_waypoint[i].size());
            std::copy(m_waypoint[i].begin(), m_waypoint[i].end(), buf_waypoint.begin());
            buf_waypoint[2] = std::to_string(std::stof(buf_waypoint[2]) + 1.0);
            waypoints.emplace_back(buf_waypoint);
        }
        lines.emplace_back(waypoints);
    }
    if (!lines.empty())
    {
        showLines(lines, colors, "error");
        ui->highlight_button->setChecked(true);
    }
}

void ScenarioEditorPlugin::makeScenarioEditIntMarker()
{
    if (m_scenario.size() == 0 || m_waypoint.empty() || m_id_score.empty()) return;

    clearAllIntMarkers();
    std::vector<std::vector<std::string>> waypoints;
    std::vector<std::vector<float>> colors;
    std::vector<std::string> id_list;

    int i = 0;
    for (const auto &scenario : m_scenario)
    {
        if (m_layer*m_layer_size <= std::stoi((std::string)scenario["start_id"]) && std::stoi((std::string)m_scenario[i]["end_id"]) < (m_layer+1)*m_layer_size-1 )
        {
            waypoints.emplace_back(m_waypoint[std::stoi((std::string)scenario["start_id"])]);
            id_list.emplace_back(std::to_string(i)+"-start");
            colors.emplace_back(m_color_list[i%m_color_list.size()]);
            waypoints.emplace_back(m_waypoint[std::stoi((std::string)scenario["end_id"])]);
            id_list.emplace_back(std::to_string(i)+"-end");
            colors.emplace_back(m_color_list[i%m_color_list.size()]);
        }
        i++;
    }
    makeIntScenarios(id_list, waypoints, colors);

    waypoints.clear();
    colors = {{1.0, 0.7, 0.4, 1.0}};

    for (int i = m_layer*m_layer_size; i < (m_layer+1)*m_layer_size; i++)
    {
        if (i > m_waypoint.size()-1) break;
        waypoints.emplace_back(m_waypoint[i]);
    }
    showArrows(waypoints, colors[0]);
}

void ScenarioEditorPlugin::makeWaypointIntMarker()
{
    if (m_scenario.size() == 0 || m_waypoint.empty() || m_id_score.empty()) return;

    clearAllIntMarkers();
    std::vector<std::vector<std::string>> waypoints;
    std::vector<std::vector<float>> colors = {{0.0, 1.0, 0.0, 1.0}};
    std::vector<std::string> id_list;
    for (int i = m_layer*m_layer_size; i < (m_layer+1)*m_layer_size; i++)
    {
        if (i > m_waypoint.size()-1) break;
        waypoints.emplace_back(m_waypoint[i]);
        id_list.emplace_back(std::to_string(i));
    }
    makeButtonWaypoints(id_list, waypoints, colors);
}

void ScenarioEditorPlugin::scenarioCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    {
        int waypoint_index = getClosestWaypoint(m_waypoint, feedback->pose, m_layer*m_layer_size, std::min((m_layer+1)*m_layer_size, (int)m_waypoint.size()-1));
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

        QStringList id = QString::fromStdString(feedback->marker_name).split("-");
        if (id.size() == 2)
        {
            if (id.at(1).compare("start", Qt::CaseSensitive) == 0)
            {
                m_scenario[id[0].toInt()]["start_id"] = std::to_string(waypoint_index);
                m_selected_scenario_id = id[0].toInt();
                updatePanel(m_selected_scenario_id);
                updateHighlight();
            }
            else if (id.at(1).compare("end", Qt::CaseSensitive) == 0)
            {
                m_scenario[id[0].toInt()]["end_id"] = std::to_string(waypoint_index);
                m_selected_scenario_id = id[0].toInt();
                updatePanel(m_selected_scenario_id);
                updateHighlight();
            }
        }
    }
    else if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
    {
        QStringList id = QString::fromStdString(feedback->marker_name).split("-");
        if (id.size() == 2)
        {
            m_selected_scenario_id = id[0].toInt();
            updatePanel(m_selected_scenario_id);
        }
    }
}

void ScenarioEditorPlugin::buttonCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    clock_t fb_time = clock();
    if (float(fb_time - m_last_fb_time) / CLOCKS_PER_SEC < 0.5) return;
    m_last_fb_time = fb_time;

    if (m_is_add_scenario_mode)
    {
        if (ui->scenario_start_text->text().compare("---", Qt::CaseSensitive) == 0)
        {
            ui->scenario_start_label->setStyleSheet("background-color : transparent;");
            ui->scenario_start_text->setStyleSheet("background-color : transparent;");
            ui->scenario_end_label->setStyleSheet("background-color : rgb(100, 200, 0);");
            ui->scenario_end_text->setStyleSheet("background-color : rgb(100, 200, 0);");
            ui->scenario_start_text->setText(QString::fromStdString(feedback->marker_name));
        }
        else if (ui->scenario_end_text->text().compare("---", Qt::CaseSensitive) == 0)
        {
            ui->scenario_end_text->setText(QString::fromStdString(feedback->marker_name));
            ui->add_scenario_botton->setChecked(false);
            ui->add_scenario_botton->setText("Add Scenario");
            m_selected_scenario_id = insertNewScenario(ui->scenario_start_text->text().toStdString(), feedback->marker_name);

            m_is_add_scenario_mode = false;
            updatePanel(m_selected_scenario_id);
            makeScenarioEditIntMarker();
        }
    }
}

int ScenarioEditorPlugin::insertNewScenario(const std::string &start, const std::string &end)
{
    for (auto itr = m_scenario.begin(); itr != m_scenario.end(); itr++)
    {
        if (std::stoi((std::string)(*itr)["start_id"]) > std::stoi(start))
        {
            try
            {
                json j;
                j["start_id"] = start;
                j["end_id"] = end;
                j["speed_limit"] = std::to_string(0);
                j["errors"] = json::array();
                m_scenario.insert(itr, j);
                return std::distance(m_scenario.begin(), itr);
            }
            catch (char *error)
            {
                std::cout << error << std::endl;
                return m_scenario.size()-1;
            }
        }
    }
}

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(scenario_editor_plugin::ScenarioEditorPlugin, rviz::Panel)
