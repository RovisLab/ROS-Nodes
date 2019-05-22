#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QProcess>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

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

private:
    Ui::MainWindow *ui;

    QProcess *myStartProcess;
    QProcess *myStopProcess;
    QProcess *rqtGraphProcess;
    // Map image
    QString mapSource;
    QPixmap *mapImage;
    QProcess *saveMapProcess;


    QString packagePath;
    QString shellScriptsPath;
    QString defaultPoseFileFolder;
    QString outputPoseYamlPath;
    // shell script args
    QString simulatedMappingScriptPath;
    QString simulatedNavigationScriptPath;
    QString rqtGraphScriptPath;
    QString mapSaverScriptPath;
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

#endif // MAINWINDOW_H
