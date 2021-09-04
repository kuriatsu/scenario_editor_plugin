#ifndef AIDISCENARIOEDITOR_H
#define AIDISCENARIOEDITOR_H

#include <QWidget>
#include <QFileDialog>
#include <fstream>
#include <sstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace Ui {
class AidiScenarioEditor;
}

class AidiScenarioEditor : public QWidget
{
    Q_OBJECT

public:
    explicit AidiScenarioEditor(QWidget *parent = 0);
    ~AidiScenarioEditor();

private slots:
    void on_waypoint_button_clicked();

    void on_scenario_button_clicked();

    void on_data_ok_clicked();

    void on_id_score_button_clicked();

    void on_id_score_edit_select_currentIndexChanged(int index);

    void on_id_score_description_select_currentIndexChanged(const QString &arg1);

private:
    Ui::AidiScenarioEditor *ui;
    QString m_waypoint_file;
    QString m_scenario_file;
    QString m_id_score_file;
    std::vector<std::vector<std::string>> m_id_score;
    std::vector<std::vector<std::string>> m_waypoint;
    json m_scenario;

    void read_csv(const std::string filename, auto& out_list);
    void set_id_score(const auto& id_score_list);
    void read_json(const std::string filename, auto& out_list);
};

#endif // AIDISCENARIOEDITOR_H
