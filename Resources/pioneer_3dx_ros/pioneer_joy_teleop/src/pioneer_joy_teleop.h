#ifndef PIONEER_JOY_TELEOP_H
#define PIONEER_JOY_TELEOP_H

#include <QWidget>
#include <QProcess>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>

namespace Ui {
class Pioneer_Joy_Teleop;
}

class Pioneer_Joy_Teleop : public QWidget
{
    Q_OBJECT

public:
    explicit Pioneer_Joy_Teleop(QWidget *parent = 0);
    ~Pioneer_Joy_Teleop();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_pressed();

private:
    Ui::Pioneer_Joy_Teleop *ui;
    ros::Publisher cmd_vel_publisher;
    QString packagePath;
    geometry_msgs::Twist cmdVelMsg;
    QPixmap *worldImage;
    QProcess *startJoyProcess;
    QProcess *killerNode;
};

#endif // PIONEER_JOY_TELEOP_H
