#ifndef REAL_H
#define REAL_H

#include <QWidget>
#include <QProcess>

namespace Ui {
class Real;
}

class Real : public QWidget
{
    Q_OBJECT

public:
    explicit Real(QWidget *parent = nullptr);
    ~Real();

private slots:

    void on_doubleSpinBox_yawPoseMapping_valueChanged(double arg1);

    void on_dial_yawPoseMapping_valueChanged(int value);

    void on_pushButton_openPoseFileMapping_clicked();

    void on_lineEdit_poseFileMapping_returnPressed();

    void on_pushButton_startMappingTool_clicked();

    void on_pushButton_openTeleopMapping_clicked();

    void on_pushButton_openRqtGraphMapping_clicked();

    void on_pushButton_saveMap_clicked();

    void on_pushButton_stopMappingTool_clicked();

    void on_comboBox_worldsNavigation_currentIndexChanged(const QString &arg1);

    void on_doubleSpinBox_yaw1PoseNavigation_valueChanged(double arg1);

    void on_dial_yaw1PoseNavigation_valueChanged(int value);

    void on_doubleSpinBox_yaw2PoseNavigation_valueChanged(double arg1);

    void on_dial_yaw2PoseNavigation_valueChanged(int value);

    void on_doubleSpinBox_yaw3PoseNavigation_valueChanged(double arg1);

    void on_dial_yaw3PoseNavigation_valueChanged(int value);

    void on_doubleSpinBox_yaw4PoseNavigation_valueChanged(double arg1);

    void on_dial_yaw4PoseNavigation_valueChanged(int value);

    void on_doubleSpinBox_yaw5PoseNavigation_valueChanged(double arg1);

    void on_dial_yaw5PoseNavigation_valueChanged(int value);

    void on_pushButton_openPoseFileNavigation_clicked();

    void on_lineEdit_poseFileNavigation_returnPressed();

    void on_pushButton_startNavigationTool_clicked();

    void on_pushButton_openRqtGraphNav_clicked();

    void on_pushButton_openRqtReconfigureNav_clicked();

    void on_pushButton_stopNavigationTool_clicked();

    void complete_usernames_from_yaml(const QString& filePath, const QString& module);

    void complete_coordinates_from_yaml(const QString& filePath, const QString& module);

    void create_yaml_from_usernames(const QString& filePath, const QString& module, const int& nrOfUsedRobots=1);

    void create_yaml_from_coordinates(const QString& filePath, const QString& module, const int& nrOfUsedRobots=1);

    void findAndDestroy(QProcess *startedProcess,const QString& createdYamlFilePath);

    void oneShotProcess(const QString& package, const QString& node, const QString& optionalArgument1="", const QString& optionalArgument2="");


    void on_pushButton_openUsernameFileMapping_clicked();

    void on_lineEdit_usernameFileMapping_returnPressed();

    void on_pushButton_openUsernameFileNavigation_clicked();

private:
    Ui::Real *ui;
    // Paths
    QString packagePath;
    QString realMappingScriptPath;
    QString realNavigationScriptPath;

    QString worldImagePath;

    QString poseFileDefaultPath;
    QString poseFilePathFromMapping;
    QString poseFilePathFromNavigation;
    QString usedPoseFilePathForMappig;
    QString usedPoseFilePathForNavigation;
    QString usedUsernameFilePathForMapping;
    QString usedUsernameFilePathForNavigation;
    QString pidTxtFilePath;

    //images
    QPixmap * worldImage;

    // QProcess
    QProcess *moduleStartProcess;
    QProcess *saveMapProcess;
    QProcess *pioneerTeleopKeyProcess;
    QProcess *rqtGraphProcess;
    QProcess *godKillerProcess;

    //launchfiles arguments
    QString worldArgument;
    QString robot_URDF_modelArgument;
    QString kinect_to_laserscanArgument;
    QString gmapping_config_typeArgument;
    QString pose_fileArgument;
    QString localization_typeArgument;
    QString base_global_plannerArgument;
    QString base_local_plannerArgument;
    QString rviz_configArgument;
    QString robot_names_fileArgument;

    //script commands
    QString realMappingScriptCommand;
    QString realNavigationScriptCommand;

    int numberOfUsedRobots;
};

#endif // REAL_H
