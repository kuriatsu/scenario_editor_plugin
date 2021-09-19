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
}

void ScenarioEditorPlugin::on_id_score_button_pressed()
{
    m_id_score_file = QFileDialog::getOpenFileName(this, tr("open file"), "~/", tr("csv file (*csv)"));
    ui->id_score_text->setText(m_id_score_file);
}

void ScenarioEditorPlugin::on_scenario_button_pressed()
{
    m_scenario_file = QFileDialog::getOpenFileName(this, tr("open file"), "~/", tr("json file (*json)"));
    ui->scenario_text->setText(m_scenario_file);
}


void ScenarioEditorPlugin::read_csv(const std::string filename, std::vector<std::vector<std::string>> &out_list)
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

void ScenarioEditorPlugin::read_json(const std::string filename, json &out_list)
{
    std::ifstream ifs(filename);
    ifs >> out_list;
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
        read_csv(m_waypoint_file.toStdString(), m_waypoint);
        read_csv(m_id_score_file.toStdString(), m_id_score);
        read_json(m_scenario_file.toStdString(), m_scenario);
        ui->layer_value->setMaximum(m_waypoint.size()/m_layer_size);
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
        addScenarioMode();
    }
    else
    {
        ui->add_scenario_botton->setText(tr("Add Scenario"));
        ui->scenario_start_label->setStyleSheet("background-color : transparent;");
        ui->scenario_start_text->setStyleSheet("background-color : transparent;");
        ui->scenario_start_label->setStyleSheet("background-color : transparent;");
        ui->scenario_start_text->setStyleSheet("background-color : transparent;");
        ui->current_scenario_text->setText(tr("---"));
        ui->scenario_start_text->setText(tr("---"));
        ui->scenario_end_text->setText(tr("---"));
        scenarioEditMode();
    }

}

void ScenarioEditorPlugin::on_remove_scenario_button_pressed()
{
    if (ui->current_scenario_text->text().compare("---", Qt::CaseSensitive) == 0) return;
    m_scenario.erase(m_scenario.begin()+std::stoi(ui->current_scenario_text->text().toStdString()));
    ui->current_scenario_text->setText(tr("---"));
    ui->scenario_start_text->setText(tr("---"));
    ui->scenario_end_text->setText(tr("---"));
    scenarioEditMode();
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
                scenarioEditMode();
                setScenarioInfo(0);
            }
            break;
    }
}

void ScenarioEditorPlugin::on_layer_size_value_editingFinished()
{
    m_layer_size = ui->layer_size_value->value();
    ui->layer_value->setMaximum(m_waypoint.size()/m_layer_size);
    if (ui->add_scenario_botton->isChecked())
    {
        addScenarioMode();
    }
    else
    {
        scenarioEditMode();
    }
}

void ScenarioEditorPlugin::on_layer_value_valueChanged(int arg1)
{
    m_layer = arg1;
    if (ui->add_scenario_botton->isChecked())
    {
        addScenarioMode();
    }
    else
    {
        scenarioEditMode();
    }
}


void ScenarioEditorPlugin::on_candidate_list_itemDoubleClicked(QListWidgetItem *item)
{
    ui->candidate_list->takeItem(ui->candidate_list->row(item));
    ui->included_list->addItem(item->text());
    ui->candidate_list->sortItems(Qt::AscendingOrder);
    m_scenario[ui->current_scenario_text->text().toInt()]["errors"].emplace_back(item->text().toStdString());
    delete item;
}

void ScenarioEditorPlugin::on_included_list_itemDoubleClicked(QListWidgetItem *item)
{
    ui->included_list->takeItem(ui->included_list->row(item));
    ui->candidate_list->addItem(item->text());
    ui->candidate_list->sortItems(Qt::AscendingOrder);

    auto &errors = m_scenario[ui->current_scenario_text->text().toInt()]["errors"];
    for (auto itr = errors.begin(); itr != errors.end(); ++itr)
    {
        if (*itr == item->text().toStdString())
        {
            errors.erase(itr);
            delete item;
            return;
        }
    }
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
            // ui->description_text->setText(QString::fromLocal8Bit(id_score[6].c_str()));
            break;
        }
    }
}


void ScenarioEditorPlugin::on_speed_value_editingFinished()
{
    if (ui->current_scenario_text->text().compare("---", Qt::CaseSensitive)==0) return;
    m_scenario[ui->current_scenario_text->text().toInt()]["speed_limit"] = ui->speed_value->value();
}

void ScenarioEditorPlugin::on_highlight_button_clicked(bool checked)
{

    if (checked)
    {
        QString target_error;
        if (!ui->candidate_list->selectedItems().empty())
            target_error = ui->candidate_list->selectedItems()[0]->text();
        else if (!ui->included_list->selectedItems().empty())
            target_error = ui->included_list->selectedItems().at(0)->text();
        else
        {
            ui->highlight_button->setChecked(false);
            return;
        }
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
                if (target_error.compare(QString::fromStdString(error), Qt::CaseSensitive) != 0)
                    continue;
                is_highlight = true;
            }

            if (!is_highlight) continue;
            for (int i = std::stoi((std::string)scenario["start_id"]); i <= std::stoi((std::string)scenario["end_id"]); i++)
            {
                if (i > m_waypoint.size()-1) break;
                waypoints.emplace_back(m_waypoint[i]);
            }
            lines.emplace_back(waypoints);
        }
        if (!lines.empty())
        {
            showLines(lines, colors);
            ui->highlight_button->setChecked(true);
        }
    }
    else
    {
        clearLines();
        setScenarioInfo(ui->current_scenario_text->text().toInt());
        ui->highlight_button->setChecked(false);
    }
}

void ScenarioEditorPlugin::setScenarioInfo(const int selected_scenario_id)
{
    if (m_scenario.size() == 0 || m_waypoint.empty() || m_id_score.empty()) return;

    ui->included_list->clear();
    ui->candidate_list->clear();
    ui->current_scenario_text->setText(QString::number(selected_scenario_id));
    ui->scenario_start_text->setText(QString::fromStdString(m_scenario[selected_scenario_id]["start_id"]));
    ui->scenario_end_text->setText(QString::fromStdString(m_scenario[selected_scenario_id]["end_id"]));
    ui->speed_value->setValue(std::stoi((std::string)m_scenario[selected_scenario_id]["speed_limit"]));
    if (ui->scenario_start_text->text().toInt() >= ui->scenario_end_text->text().toInt())
    {
        ui->scenario_start_label->setStyleSheet("background-color : rgb(200, 0, 0);");
        ui->scenario_start_text->setStyleSheet("background-color : rgb(200, 0, 0);");
        ui->scenario_end_label->setStyleSheet("background-color : rgb(200, 0, 0);");
        ui->scenario_end_text->setStyleSheet("background-color : rgb(200, 0, 0);");
    }
    else
    {
        ui->scenario_start_label->setStyleSheet("background-color : transparent;");
        ui->scenario_start_text->setStyleSheet("background-color : transparent;");
        ui->scenario_end_label->setStyleSheet("background-color : transparent;");
        ui->scenario_end_text->setStyleSheet("background-color : transparent;");
    }
    // add to candidate/include list
    for (std::vector<std::string> &id_score : m_id_score)
    {
        bool included_flag = false;
        for (const std::string &error : m_scenario[selected_scenario_id]["errors"])
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
    highlightScenario();
    return;
}

void ScenarioEditorPlugin::highlightScenario()
{
    clearLines();
    std::vector<std::vector<std::string>> waypoints;
    std::vector<std::vector<std::vector<std::string>>> lines;
    std::vector<std::vector<float>> colors = {{1.0, 0.5, 0.5, 1.0}};
    for (int i = ui->scenario_start_text->text().toInt(); i <= ui->scenario_end_text->text().toInt(); i++)
    {
        if (i > m_waypoint.size()-1) break;
        waypoints.emplace_back(m_waypoint[i]);
    }
    lines.emplace_back(waypoints);
    showLines(lines, colors);
}

void ScenarioEditorPlugin::scenarioEditMode()
{
    if (m_scenario.size() == 0 || m_waypoint.empty() || m_id_score.empty()) return;
    clearLines();
    clearPoints();
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

void ScenarioEditorPlugin::addScenarioMode()
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
                ui->current_scenario_text->setText(id[0]);
                setScenarioInfo(id[0].toInt());
            }
            else if (id.at(1).compare("end", Qt::CaseSensitive) == 0)
            {
                m_scenario[id[0].toInt()]["end_id"] = std::to_string(waypoint_index);
                ui->current_scenario_text->setText(id[0]);
                setScenarioInfo(id[0].toInt());
            }
        }
    }
    else if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
    {
        QStringList id = QString::fromStdString(feedback->marker_name).split("-");
        if (id.size() == 2)
        {
            ui->current_scenario_text->setText(id[0]);
            setScenarioInfo(id[0].toInt());
        }
    }
}

void ScenarioEditorPlugin::buttonCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    clock_t fb_time = clock();
    if (float(fb_time - m_last_fb_time) / CLOCKS_PER_SEC < 0.5) return;
    m_last_fb_time = fb_time;

    if (ui->add_scenario_botton->text().compare("Cancel", Qt::CaseSensitive) == 0)
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
            int current_scenario = insertNewScenario(ui->scenario_start_text->text().toStdString(), feedback->marker_name);

            setScenarioInfo(current_scenario);
            scenarioEditMode();
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
