#ifndef PIONEER_KEY_TELEOP_H
#define PIONEER_KEY_TELEOP_H

#include <QWidget>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace Ui {
class Pioneer_Key_Teleop;
}

class Pioneer_Key_Teleop : public QWidget
{
    Q_OBJECT

public:
    explicit Pioneer_Key_Teleop(QWidget *parent = 0);
    ~Pioneer_Key_Teleop();

private slots:
    void on_pushButton_up_clicked();

    void on_pushButton_upRight_clicked();

    void on_pushButton_right_pressed();

    void on_pushButton_donwRight_pressed();

    void on_pushButton_down_clicked();

    void on_pushButton_downLeft_clicked();

    void on_pushButton_left_clicked();

    void on_pushButton_upLeft_clicked();

    void on_pushButton_stop_clicked();

    void on_doubleSpinBox_valueChanged(double arg1);

    void on_doubleSpinBox_2_valueChanged(double arg1);

private:
    Ui::Pioneer_Key_Teleop *ui;
    ros::Publisher chatter_publisher;
    ros::Publisher cmd_vel_publisher;
    geometry_msgs::Twist cmdVelMsg;
    double speed, turn;
    double x, th;
};

#endif // PIONEER_KEY_TELEOP_H
