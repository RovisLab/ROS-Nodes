#include "pioneer_key_teleop.h"
#include "pioneer_key_teleop/ui_pioneer_key_teleop.h"
#include <QString>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <QIcon>

Pioneer_Key_Teleop::Pioneer_Key_Teleop(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Pioneer_Key_Teleop)
{
    ui->setupUi(this);

    //speed = ui->doubleSpinBox;
    speed = ui->doubleSpinBox->value();
    turn = ui->doubleSpinBox_2->value();
    x = 0.0;
    th = 0.0;
    QIcon iconUpLeft(":/new/res/icons/upLeft.png");
    QIcon iconUp(":/new/res/icons/up.png");
    QIcon iconUpRight(":/new/res/icons/upRight.png");
    QIcon iconLeft(":/new/res/icons/left.png");
    QIcon iconStop(":/new/res/icons/stop.png");
    QIcon iconRight(":/new/res/icons/right.png");
    QIcon iconDownLeft(":/new/res/icons/downLeft.png");
    QIcon iconDown(":/new/res/icons/down.png");
    QIcon iconDownRight(":/new/res/icons/downRight.png");

    ui->pushButton_upLeft->setIcon(iconUpLeft);
    ui->pushButton_up->setIcon(iconUp);
    ui->pushButton_upRight->setIcon(iconUpRight);
    ui->pushButton_left->setIcon(iconLeft);
    ui->pushButton_stop->setIcon(iconStop);
    ui->pushButton_right->setIcon(iconRight);
    ui->pushButton_downLeft->setIcon(iconDownLeft);
    ui->pushButton_down->setIcon(iconDown);
    ui->pushButton_donwRight->setIcon(iconDownRight);

    int argc = 0; char **argv = NULL;
    ros::init(argc, argv, "pioneer_key_teleop");
    if (!ros::master::check())
    {
        ROS_INFO("No master started!");
        this->close();
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle nodeHandle_;
    topicName = "pioneer1/cmd_vel";
    cmd_vel_publisher = nodeHandle_.advertise<geometry_msgs::Twist>(topicName.toStdString(), 10);
    //ros::spin();
}

Pioneer_Key_Teleop::~Pioneer_Key_Teleop()
{
    delete ui;
}

void Pioneer_Key_Teleop::on_pushButton_up_clicked()
{
    x = 1.0;
    th = 0.0;
    cmdVelMsg.linear.x = x * speed;
    cmdVelMsg.angular.z = th * turn;
    cmd_vel_publisher.publish(cmdVelMsg);
}

void Pioneer_Key_Teleop::on_pushButton_upRight_clicked()
{
    x = 1.0;
    th = -1.0;
    cmdVelMsg.linear.x = x * speed;
    cmdVelMsg.angular.z = th * turn;
    cmd_vel_publisher.publish(cmdVelMsg);
}

void Pioneer_Key_Teleop::on_pushButton_right_pressed()
{
    x = 0.0;
    th = -1.0;
    cmdVelMsg.linear.x = x * speed;
    cmdVelMsg.angular.z = th * turn;
    cmd_vel_publisher.publish(cmdVelMsg);
}

void Pioneer_Key_Teleop::on_pushButton_donwRight_pressed()
{
    x = -1.0;
    th = 1.0;
    cmdVelMsg.linear.x = x * speed;
    cmdVelMsg.angular.z = th * turn;
    cmd_vel_publisher.publish(cmdVelMsg);
}

void Pioneer_Key_Teleop::on_pushButton_down_clicked()
{
    x = -1.0;
    th = 0.0;
    cmdVelMsg.linear.x = x * speed;
    cmdVelMsg.angular.z = th * turn;
    cmd_vel_publisher.publish(cmdVelMsg);
}

void Pioneer_Key_Teleop::on_pushButton_downLeft_clicked()
{
    x = -1.0;
    th = -1.0;
    cmdVelMsg.linear.x = x * speed;
    cmdVelMsg.angular.z = th * turn;
    cmd_vel_publisher.publish(cmdVelMsg);
}

void Pioneer_Key_Teleop::on_pushButton_left_clicked()
{
    x = 0.0;
    th = 1.0;
    cmdVelMsg.linear.x = x * speed;
    cmdVelMsg.angular.z = th * turn;
    cmd_vel_publisher.publish(cmdVelMsg);
}

void Pioneer_Key_Teleop::on_pushButton_upLeft_clicked()
{
    x = 1.0;
    th = 1.0;
    cmdVelMsg.linear.x = x * speed;
    cmdVelMsg.angular.z = th * turn;
    cmd_vel_publisher.publish(cmdVelMsg);
}

void Pioneer_Key_Teleop::on_pushButton_stop_clicked()
{
    x = 0.0;
    th = 0.0;
    cmdVelMsg.linear.x = x * speed;
    cmdVelMsg.angular.z = th * turn;
    cmd_vel_publisher.publish(cmdVelMsg);
}

void Pioneer_Key_Teleop::on_doubleSpinBox_valueChanged(double arg1)
{
    speed = arg1;
}

void Pioneer_Key_Teleop::on_doubleSpinBox_2_valueChanged(double arg1)
{
    turn = arg1;
}

void Pioneer_Key_Teleop::on_lineEdit_textEdited(const QString &arg1)
{
    ros::NodeHandle nodeHandle_;
    topicName = arg1;
    cmd_vel_publisher = nodeHandle_.advertise<geometry_msgs::Twist>(topicName.toStdString(), 10);
}
