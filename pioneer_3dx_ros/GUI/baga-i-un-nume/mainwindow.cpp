#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QPixmap>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_start_mapping_clicked()
{
    QString program = "/home/gatu270124/my_local_repo/src/ROS-Nodes/pioneer_3dx_ros/Simulations/run_pioneer_mapping.sh";
    myProcess = new QProcess(this);
    myProcess->start(program);
    //system("/home/gatu270124/my_local_repo/src/ROS-Nodes/pioneer_3dx_ros/Simulations/run_pioneer_mapping.sh");
}

void MainWindow::on_combo_box_mapping_currentIndexChanged()
{
    QPixmap pixmap("/home/gatu270124/baga-i-un-nume/5.jpg");
    pixmap.scaled(ui->img_mapping->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    ui->img_mapping->setPixmap(pixmap);

}

void MainWindow::on_pushButton_clicked()
{
    myProcess->kill();

}
