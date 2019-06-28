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
#include <QIcon>
#include <QStyle>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

Pioneer_Gui::Pioneer_Gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Pioneer_Gui)
{
    ui->setupUi(this);

    connect(ui->real, SIGNAL(clicked()), this, SLOT(show_real_tool()));
    connect(ui->sim, SIGNAL(clicked()), this, SLOT(show_simulation_tool()));

    real = new Real(this);
    sim = new Sim(this);

    ui->stackedWidget->addWidget(sim);
    ui->stackedWidget->addWidget(real);

    ui->stackedWidget->setCurrentWidget(sim);

    ui->sim->setIcon(this->style()->standardIcon(QStyle::SP_DialogApplyButton));
}

Pioneer_Gui::~Pioneer_Gui()
{
    delete ui;
}

void Pioneer_Gui::show_simulation_tool()
{
    ui->real->setIcon(QIcon());
    ui->sim->setIcon(this->style()->standardIcon(QStyle::SP_DialogApplyButton));
    ui->stackedWidget->removeWidget(sim);

    delete sim;

    sim = new Sim(this);

    ui->stackedWidget->addWidget(sim);
    ui->stackedWidget->setCurrentWidget(sim);
}

void Pioneer_Gui::show_real_tool()
{
    ui->sim->setIcon(QIcon());
    ui->real->setIcon(this->style()->standardIcon(QStyle::SP_DialogApplyButton));
    ui->stackedWidget->removeWidget(real);

    delete real;

    real = new Real(this);

    ui->stackedWidget->addWidget(real);
    ui->stackedWidget->setCurrentWidget(real);
}
