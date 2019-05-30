#include "pioneer_gui.h"
#include "ui_pioneer_gui.h"

#include <ros/ros.h>
#include <ros/package.h>

#include <QDebug>
#include <QFile>
#include <QPixmap>
#include <QMessageBox>
#include <QDir>
#include <QFileDialog>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

Pioneer_Gui::Pioneer_Gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Pioneer_Gui)
{
    ui->setupUi(this);

    real = new Real(this);
    sim = new Sim(this);
    start = new Start(this);

    ui->stackedWidget->addWidget(start);
    ui->stackedWidget->addWidget(sim);
    ui->stackedWidget->addWidget(real);

    ui->stackedWidget->setCurrentWidget(start);
}

Pioneer_Gui::~Pioneer_Gui()
{
    delete ui;
}

void Pioneer_Gui::show_simulation_tool()
{
    ui->stackedWidget->removeWidget(sim);

    delete sim;

    sim = new Sim(this);

    ui->stackedWidget->addWidget(sim);
    ui->stackedWidget->setCurrentWidget(sim);
}

void Pioneer_Gui::show_real_tool()
{
    ui->stackedWidget->removeWidget(real);

    delete real;

    real = new Real(this);

    ui->stackedWidget->addWidget(real);
    ui->stackedWidget->setCurrentWidget(real);
}

void Pioneer_Gui::back_to_main_menu()
{
    ui->stackedWidget->setCurrentWidget(start);
}

