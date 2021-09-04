#include "aidiscenarioeditor.h"
#include "ui_aidiscenarioeditor.h"

AidiScenarioEditor::AidiScenarioEditor(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::AidiScenarioEditor)
{
    ui->setupUi(this);
}

AidiScenarioEditor::~AidiScenarioEditor()
{
    delete ui;
}

void AidiScenarioEditor::on_waypoint_button_clicked()
{
    m_waypoint_file = QFileDialog::getOpenFileName(this, tr("open file"), "~/", tr("csv file (*csv)"));
    ui->waypoint_file->setText(m_waypoint_file);
}

void AidiScenarioEditor::on_scenario_button_clicked()
{
    m_scenario_file = QFileDialog::getOpenFileName(this, tr("open file"), "~/", tr("json file (*json)"));
    ui->scenario_file->setText(m_scenario_file);
}

void AidiScenarioEditor::on_id_score_button_clicked()
{
    m_id_score_file = QFileDialog::getOpenFileName(this, tr("open file"), "~/", tr("csv file (*csv)"));
    ui->id_score_text->setText(m_id_score_file);
}

void AidiScenarioEditor::on_data_ok_clicked()
{
    if(ui->scenario_button->isEnabled())
    {
        ui->scenario_button->setDisabled(true);
        ui->scenario_file->setDisabled(true);
        ui->waypoint_button->setDisabled(true);
        ui->waypoint_file->setDisabled(true);
        ui->id_score_button->setDisabled(true);
        ui->id_score_text->setDisabled(true);
        ui->data_ok->setText(tr("reload"));
    }
    else
    {
        ui->scenario_button->setDisabled(false);
        ui->scenario_file->setDisabled(false);
        ui->waypoint_button->setDisabled(false);
        ui->waypoint_file->setDisabled(false);
        ui->id_score_button->setDisabled(false);
        ui->id_score_text->setDisabled(false);
        ui->data_ok->setText(tr("ok"));
    }
    read_csv(m_waypoint_file.toStdString(), m_waypoint);
    read_csv(m_id_score_file.toStdString(), m_id_score);
    read_csv(m_scenario_file.toStdString(), m_scenario);
    set_id_score(m_id_score);

}

void AidiScenarioEditor::read_csv(const std::string filename, auto& out_list)
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

void AidiScenarioEditor::read_json(const std::string filename, auto& out_list)
{
    std::ifstream ifs(filename);
    ifs >> out_list;
}

void AidiScenarioEditor::set_id_score(const auto& id_score_list)
{
    QStringList combo_box_list;
    for (auto& id_score : id_score_list)
    {
        combo_box_list.append(QString::fromStdString(id_score[0]));
    }
    ui->id_score_edit_select->addItems(combo_box_list);
    ui->id_score_description_select->addItems(combo_box_list);
}

void AidiScenarioEditor::on_id_score_edit_select_currentIndexChanged(int index)
{
     ui->id_score_description_select->setCurrentIndex(index);
}

void AidiScenarioEditor::on_id_score_description_select_currentIndexChanged(const QString &arg1)
{
    for (std::vector<std::string>& id_score : m_id_score)
    {
        if (arg1.compare(QString::fromStdString(id_score[0]), Qt::CaseSensitive) == 0)
        {
            ui->label1_text->setText(tr(id_score[1].c_str()));
            ui->label2_text->setText(tr(id_score[2].c_str()));
            ui->label3_text->setText(tr(id_score[3].c_str()));
            ui->score_text->setText(tr(id_score[4].c_str()));
            ui->message_text->setText(tr(id_score[5].c_str()));
            ui->description_text->setText(tr(id_score[6].c_str()));
            break;
        }
    }
}
