#ifndef PIONEER_GUI_H
#define PIONEER_GUI_H

#include <QMainWindow>
#include <QProcess>
#include <ros/ros.h>
#include "start.h"
#include "real.h"
#include "sim.h"

namespace Ui {
class Pioneer_Gui;
}

class Pioneer_Gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit Pioneer_Gui(QWidget *parent = nullptr);
    ~Pioneer_Gui();

public slots:
    void show_real_tool();
    void show_simulation_tool();
    void back_to_main_menu();

private:
    Ui::Pioneer_Gui *ui;
    Start *start;
    Real *real;
    Sim *sim;
};

#endif // PIONEER_GUI_H
