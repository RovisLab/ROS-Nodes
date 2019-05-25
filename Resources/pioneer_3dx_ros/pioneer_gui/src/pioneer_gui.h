#ifndef PIONEER_GUI_H
#define PIONEER_GUI_H

#include <QMainWindow>
#include <QProcess>
#include <ros/ros.h>

namespace Ui {
class Pioneer_Gui;
}

class Pioneer_Gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit Pioneer_Gui(QWidget *parent = nullptr);
    ~Pioneer_Gui();

private slots:
    void on_start_mapping_clicked();
    void on_combo_box_mapping_currentIndexChanged();
    void on_stopButon_clicked();
    void on_save_map_clicked();


    void on_combo_box_nav_currentIndexChanged();

    void on_start_nav_clicked();

    void on_stop_nav_buton_clicked();

    void on_pushButtonRqtGraph_clicked();

    void on_pushButtonRqtGraphNav_clicked();

    void on_pushButton_clicked();

    void on_start_teleop_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_open_pose_map_clicked();

private:
    Ui::Pioneer_Gui *ui;

    QProcess *myStartProcess;
    QProcess *myStopProcess;
    QProcess *rqtGraphProcess;
    QProcess *pioneerKeyTeleopProcess;
    // Map image
    QString mapSource;
    QPixmap *mapImage;
    QProcess *saveMapProcess;


    QString packagePath;
    QString shellScriptsPath;
    QString defaultPoseFileFolder;
    QString outputPoseYamlPath;
    QString outputPoseMapYamlPath;
    // shell script args
    QString simulatedMappingScriptPath;
    QString simulatedNavigationScriptPath;
    QString rqtGraphScriptPath;
    QString mapSaverScriptPath;
    QString pioneerKeyTeleopPath;
    QString worldGazebo;
    QString robot_URDF_model;
    QString gmappingConfigType;
    QString localizationType;
    QString environmentType;
    QString baseGlobalPlanner;
    QString baseLocalPlanner;
    QString RVizConfigType;
    QString useRealKinect;
    QString kinectToLaserScan;
    QString useMPC;

    QString numberOfUsedRobots;
    QString poseFileName;

};

#endif // PIONEER_GUI_H
