#include "scenario_editor.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    AidiScenarioEditor w;
    w.show();

    return a.exec();
}
