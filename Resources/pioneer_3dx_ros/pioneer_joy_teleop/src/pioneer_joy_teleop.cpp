#include "pioneer_joy_teleop.h"
#include "pioneer_joy_teleop/ui_pioneer_joy_teleop.h"
#include <QPixmap>
#include <QString>
#include <QDebug>
#include <QFile>
#include <QMessageBox>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <QIcon>

Pioneer_Joy_Teleop::Pioneer_Joy_Teleop(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Pioneer_Joy_Teleop)
{
    ui->setupUi(this);

    packagePath = QString::fromStdString(ros::package::getPath("pioneer_joy_teleop"));
    QString worldImagePath = packagePath + "/resources/joy.png";
    qDebug() << worldImagePath;
    worldImage = new QPixmap();
    worldImage->load(worldImagePath);
    ui->label_joyImage->setScaledContents(true);
    ui->label_joyImage->setPixmap(*worldImage);

//    int argc = 0; char **argv = NULL;
//    ros::init(argc, argv, "pioneer_joy_teleop");
//    if (!ros::master::check())
//    {
//        ROS_INFO("No master started!");
//        this->close();
//    }
//    ros::start(); // explicitly needed since our nodehandle is going out of scope.
//    ros::NodeHandle nodeHandle_;
//    topicName = "pioneer1/cmd_vel";
//    cmd_vel_publisher = nodeHandle_.advertise<geometry_msgs::Twist>(topicName.toStdString(), 10);
    //ros::spin();
}

Pioneer_Joy_Teleop::~Pioneer_Joy_Teleop()
{
    delete ui;
}

void Pioneer_Joy_Teleop::on_pushButton_clicked()
{
    QString scriptPath = QString::fromStdString(ros::package::getPath("pioneer_joy_teleop")) + "/resources/pioneer_joy.sh";
    QString topic = ui->lineEdit->text();
    startJoyProcess = new QProcess(this);
    startJoyProcess->startDetached("/bin/bash", QStringList() << scriptPath << "--topic" << topic);
    startJoyProcess->waitForFinished();
    startJoyProcess->kill();
    ui->label_status->setText("Status: Active");
}

void Pioneer_Joy_Teleop::on_pushButton_2_pressed()
{
    QString pidTxt = (packagePath.left(packagePath.indexOf("/",6) + 1)) + "script_pid_joy.txt";
    QFile pidTxtFile(pidTxt);
    QString line;
    if(!pidTxtFile.open(QIODevice::ReadOnly))
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("ERROR");
        msgBox.setText("You did not started the module");
        msgBox.exec();
        return;
    }

    QTextStream in(&pidTxtFile);

    while(!in.atEnd()) {
        line = in.readLine();
    }

    QString killPidsCommand = "kill -2" + line;
    qDebug() << killPidsCommand;
    killerNode = new QProcess(this);
    killerNode->start(killPidsCommand);
    killerNode->waitForFinished();
    killerNode->kill();
    startJoyProcess->kill();
    pidTxtFile.close();
    pidTxtFile.remove();
    ui->label_status->setText("Status: Inactive");
}
