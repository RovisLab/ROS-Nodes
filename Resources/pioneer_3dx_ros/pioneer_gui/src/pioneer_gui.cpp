#include "pioneer_gui.h"
#include "pioneer_gui/ui_pioneer_gui.h"

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
    qDebug() << ui->combo_box_mapping->itemText(0);

    mapSource = ":/res/resources/worlds_jpg/" + ui->combo_box_mapping->itemText(ui->combo_box_mapping->currentIndex()) +".jpg";
    mapImage = new QPixmap();
    mapImage->load(mapSource);

    ui->img_mapping->setScaledContents(true);
    ui->img_nav->setScaledContents(true);
    ui->img_mapping->setPixmap(*mapImage);
    ui->img_nav->setPixmap(*mapImage);

    packagePath = QString::fromStdString(ros::package::getPath("pioneer_gui"));
//    qDebug() << "package_path" << packagePath;
    QString workspacePath = packagePath.left(packagePath.indexOf("/src/ROS-Nodes"));
//    qDebug() << "workspace_path:" << workspacePath;
    shellScriptsPath = packagePath + "/resources/shell_scripts/";
//    qDebug() << "shell_scripts_path" << shellScriptsPath;

    // shell script args
    simulatedMappingScriptPath = shellScriptsPath + "simulated_mapping.sh";
    simulatedNavigationScriptPath = shellScriptsPath + "simulated_navigation.sh";
    rqtGraphScriptPath = shellScriptsPath +"rqt_graph.sh";
    mapSaverScriptPath = shellScriptsPath + "map_saver.sh";
    pioneerKeyTeleopPath = shellScriptsPath + "pioneer_key_teleop.sh";
    defaultPoseFileFolder = packagePath.left(packagePath.indexOf("pioneer_gui")) + "pioneer_description/params";
    outputPoseYamlPath= defaultPoseFileFolder + "/created_yaml_poses.yaml";
    outputPoseMapYamlPath = defaultPoseFileFolder + "/one_pioneer_mapping_pose.yaml";
}

Pioneer_Gui::~Pioneer_Gui()
{
    delete ui;
}

void Pioneer_Gui::on_start_mapping_clicked()
{
    //Setting bash script arguments
    worldGazebo = ui->combo_box_mapping->currentText();

    if(ui->kinect_gmapping->isChecked())
    {
        robot_URDF_model = "pioneer_kinect";
        kinectToLaserScan = "true";
        gmappingConfigType = "gmapping_kinect";
    }
    else if(ui->hokuyo_gmapping->isChecked())
    {
        robot_URDF_model = "pioneer_hokuyo";
        kinectToLaserScan = "false";
        gmappingConfigType = "gmapping_hokuyo";
    }

    //create yaml
    QFile outputPoseYamlFile(outputPoseMapYamlPath);
    YAML::Emitter out;
    out << YAML::BeginMap;
    QString pioneer_name = "pioneer1";
    QString xString = QString::number(ui->doubleSpinBox_x_map->value());
    QString yString = QString::number(ui->doubleSpinBox_y_map->value());
    QString aString = QString::number(ui->doubleSpinBox_a_map->value());
    out << YAML::Key << pioneer_name.toStdString();
    out << YAML::Value << YAML::BeginMap << YAML::Key << "x" <<YAML::Value << xString.toStdString() << YAML::Key << "y" <<YAML::Value << yString.toStdString() << YAML::Key << "a" <<YAML::Value << aString.toStdString() << YAML::EndMap;
    out << YAML::EndMap;
    if (outputPoseYamlFile.open(QIODevice::ReadWrite))
    {
        QTextStream outputStream( &outputPoseYamlFile );
        outputStream << out.c_str();
    }

    poseFileName = "one_pioneer_mapping_pose";

    QString simMappingCommand = simulatedMappingScriptPath + " --world " + worldGazebo;
    simMappingCommand += " --robot_URDF_model " + robot_URDF_model;
    simMappingCommand += " --pose_file " + poseFileName;
    simMappingCommand += " --kinect_to_laserscan " + kinectToLaserScan;
    simMappingCommand += " --gmapping_config_type " + gmappingConfigType;
    qDebug() << simMappingCommand;

    myStartProcess = new QProcess(this);
    myStartProcess->startDetached("/bin/bash", QStringList() << simulatedMappingScriptPath << "--world" << worldGazebo <<
                                  "--robot_URDF_model" << robot_URDF_model <<
                                  "--pose_file" << poseFileName <<
                                  "--kinect_to_laserscan" << kinectToLaserScan <<
                                  "--gmapping_config_type" << gmappingConfigType);

}
void Pioneer_Gui::on_combo_box_mapping_currentIndexChanged()
{
    mapSource = ":/res/resources/worlds_jpg/" + ui->combo_box_mapping->itemText(ui->combo_box_mapping->currentIndex()) +".jpg";
    delete mapImage;
    mapImage = new QPixmap();
    mapImage->load(mapSource);
    ui->img_mapping->setScaledContents(true);
    ui->img_mapping->setPixmap(*mapImage);

    qDebug() << ui->combo_box_mapping->itemText(ui->combo_box_mapping->currentIndex());
}
void Pioneer_Gui::on_pushButton_open_pose_map_clicked()
{
    QString poseFileSourcePath = QFileDialog::getOpenFileName(this,
        tr("Open YAML file"), defaultPoseFileFolder, tr("YAML Files (*.yaml)"));
//    qDebug() << poseFileSourcePath;
    ui->lineEditPoseFile_map->setText(poseFileSourcePath);

//    qDebug() << poseFileSourcePath;

    QFile inputFile(poseFileSourcePath);
    QString line;
    if (inputFile.open(QIODevice::ReadOnly))
    {
       QTextStream in(&inputFile);
       while (!in.atEnd())
       {
          line += in.readLine();
       }
       inputFile.close();
    }
//    qDebug()<<line;

    QRegExp rx("([+-]?\\d+)\\.(\\d+)");

    QList<QString> list;
    int pos = 0;

    while ((pos = rx.indexIn(line, pos)) != -1) {
        list << rx.cap(1) + "." + rx.cap(2);
        pos += rx.matchedLength();
    }

//    qDebug() << list;

    if(list.size() < 3) {
        QMessageBox msgBox;
        msgBox.setWindowTitle("ERROR");
        msgBox.setText("The YAML file is not well configured");
        msgBox.setInformativeText("Please select the propper configuration file.");
        msgBox.exec();
        ui->doubleSpinBox_x_map->setValue(0);
        ui->doubleSpinBox_y_map->setValue(0);
        ui->doubleSpinBox_a_map->setValue(0);
    }
    else {
        ui->doubleSpinBox_x_map->setValue(list[0].toDouble());
        ui->doubleSpinBox_y_map->setValue(list[1].toDouble());
        ui->doubleSpinBox_a_map->setValue(list[2].toDouble());
    }
}
void Pioneer_Gui::on_save_map_clicked()
{
    saveMapProcess = new QProcess(this);
    if(ui->lineEditCreatedMapName->text().isEmpty())
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("ERROR");
        msgBox.setText("You did not choose a name for the created map!");
        msgBox.setInformativeText("Please set the name in the corresponding label.");
        msgBox.exec();
    }
    QString createdMapPath = packagePath.left(packagePath.indexOf("pioneer_gui")) + "pioneer_gazebo/map/" + ui->lineEditCreatedMapName->text();
    saveMapProcess = new QProcess(this);
    saveMapProcess->startDetached("/bin/bash", QStringList() << mapSaverScriptPath << createdMapPath);
    qDebug() << createdMapPath;
}
void Pioneer_Gui::on_pushButtonRqtGraph_clicked()
{
    rqtGraphProcess = new QProcess(this);
    rqtGraphProcess->startDetached("/bin/bash", QStringList() << rqtGraphScriptPath);
    rqtGraphProcess->waitForFinished();
    rqtGraphProcess->kill();
}
void Pioneer_Gui::on_start_teleop_clicked()
{
    pioneerKeyTeleopProcess = new QProcess(this);
    pioneerKeyTeleopProcess->startDetached("/bin/bash", QStringList() << pioneerKeyTeleopPath);
    pioneerKeyTeleopProcess->waitForFinished();
    pioneerKeyTeleopProcess->kill();
}
void Pioneer_Gui::on_stopButon_clicked()
{
    QString stopScriptPath = QDir::homePath() + "/script_pid.txt";
    QFile file(stopScriptPath);
    QFile outputPoseYamlFile(outputPoseMapYamlPath);
    QString line;
    if(!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "n-am apucat sa fac fisierul" << file.errorString();
    }

    QTextStream in(&file);

    while(!in.atEnd()) {
        line = in.readLine();
    }

    QString comanda = "kill -2 " + line;
    qDebug() << comanda;

    myStopProcess = new QProcess(this);
    myStopProcess->start(comanda);
    myStopProcess->waitForFinished();
    myStopProcess->kill();
    myStartProcess->kill();
    file.close();
    file.remove();
    outputPoseYamlFile.remove();
}

void Pioneer_Gui::on_start_nav_clicked()
{
    // set args
    worldGazebo = ui->combo_box_nav->currentText();

    if(ui->kinect_nav->isChecked())
    {
        robot_URDF_model = "pioneer_kinect";
        kinectToLaserScan = "true";
    }
    else if(ui->hokuyo_nav->isChecked())
    {
        robot_URDF_model = "pioneer_hokuyo";
        kinectToLaserScan = "false";
    }
    // How many robots are used
    if(ui->pioneer1->isChecked())
    {
        numberOfUsedRobots = "1";
    }
    else if (ui->pioneer2->isChecked())
    {
        numberOfUsedRobots = "2";
    }
    else if (ui->pioneer3->isChecked())
    {
        numberOfUsedRobots = "3";
    }
    else if (ui->pioneer4->isChecked())
    {
        numberOfUsedRobots = "4";
    }
    else if (ui->pioneer5->isChecked())
    {
        numberOfUsedRobots = "5";
    }

    QFile outputPoseYamlFile(outputPoseYamlPath);
    //create yaml
    YAML::Emitter out;
    out << YAML::BeginMap;
    for(qint8 i = 1; i <= numberOfUsedRobots.toInt(); ++i) {
        QString pioneer_name;
        QString xString;
        QString yString;
        QString aString;
        switch (i) {
            case 1: {
                pioneer_name = "pioneer" + QString::number(i);
                xString = QString::number(ui->x1->value());
                yString = QString::number(ui->y1->value());
                aString = QString::number(ui->a1->value());
                break;
            }
            case 2: {
                pioneer_name = "pioneer" + QString::number(i);
                xString = QString::number(ui->x2->value());
                yString = QString::number(ui->y2->value());
                aString = QString::number(ui->a2->value());
                break;
            }
            case 3: {
                pioneer_name = "pioneer" + QString::number(i);
                xString = QString::number(ui->x3->value());
                yString = QString::number(ui->y3->value());
                aString = QString::number(ui->a3->value());
                break;
            }
            case 4: {
                pioneer_name = "pioneer" + QString::number(i);
                xString = QString::number(ui->x4->value());
                yString = QString::number(ui->y4->value());
                aString = QString::number(ui->a4->value());
                break;
            }
            case 5: {
                pioneer_name = "pioneer" + QString::number(i);
                xString = QString::number(ui->x5->value());
                yString = QString::number(ui->y5->value());
                aString = QString::number(ui->a5->value());
                break;
            }
        }
        out << YAML::Key << pioneer_name.toStdString();
        out << YAML::Value << YAML::BeginMap << YAML::Key << "x" <<YAML::Value << xString.toStdString() << YAML::Key << "y" <<YAML::Value << yString.toStdString() << YAML::Key << "a" <<YAML::Value << aString.toStdString() << YAML::EndMap;
    }
    out << YAML::EndMap;
    if (outputPoseYamlFile.open(QIODevice::ReadWrite))
    {
        QTextStream outputStream( &outputPoseYamlFile );
        outputStream << out.c_str();
    }
    poseFileName = "created_yaml_poses";

    localizationType = ui->comboBox_localization_type->itemText(ui->comboBox_localization_type->currentIndex());

    baseGlobalPlanner = ui->comboBox_global_planner->itemText(ui->comboBox_global_planner->currentIndex());
    baseLocalPlanner = ui->comboBox_local_planner->itemText(ui->comboBox_local_planner->currentIndex());

    RVizConfigType = numberOfUsedRobots + "_robots_navigation_" + baseGlobalPlanner + "_" + baseLocalPlanner;

    QString simNavCommand = simulatedNavigationScriptPath + " --world " + worldGazebo;
    simNavCommand += " --robot_URDF_model " + robot_URDF_model;
    simNavCommand += " --pose_file " + poseFileName;
    simNavCommand += " --participants " + numberOfUsedRobots;
    simNavCommand += " --kinect_to_laserscan " + kinectToLaserScan;
    simNavCommand += " --localization_type " + localizationType;
    simNavCommand += " --base_global_planner " + baseGlobalPlanner;
    simNavCommand += " --base_local_planner " + baseLocalPlanner;
    simNavCommand += "--rviz_config " + RVizConfigType;
    qDebug() << simNavCommand;

    myStartProcess = new QProcess(this);
    myStartProcess->startDetached("/bin/bash", QStringList() << simulatedNavigationScriptPath <<
                                  "--world" << worldGazebo <<
                                  "--robot_URDF_model" << robot_URDF_model <<
                                  "--pose_file" << poseFileName <<
                                  "--participants" << numberOfUsedRobots <<
                                  "--kinect_to_laserscan" << kinectToLaserScan <<
                                  "--localization_type" << localizationType <<
                                  "--base_global_planner" <<baseGlobalPlanner <<
                                  "--base_local_planner" << baseLocalPlanner <<
                                  "--rviz_config" << RVizConfigType);

}
void Pioneer_Gui::on_combo_box_nav_currentIndexChanged()
{
    mapSource = ":/res/resources/worlds_jpg/" + ui->combo_box_nav->itemText(ui->combo_box_nav->currentIndex()) +".jpg";

    delete mapImage;
    mapImage = new QPixmap();
    mapImage->load(mapSource);
    ui->img_nav->setScaledContents(true);
    ui->img_nav->setPixmap(*mapImage);

    qDebug() << ui->combo_box_nav->itemText(ui->combo_box_nav->currentIndex());
}
void Pioneer_Gui::on_pushButton_clicked()
{

    QString poseFileSourcePath = QFileDialog::getOpenFileName(this,
        tr("Open YAML file"), defaultPoseFileFolder, tr("YAML Files (*.yaml)"));
//    qDebug() << poseFileSourcePath;
    ui->lineEditPoseFile->setText(poseFileSourcePath);

//    qDebug() << poseFileSourcePath;

    QFile inputFile(poseFileSourcePath);
    QString line;
    if (inputFile.open(QIODevice::ReadOnly))
    {
       QTextStream in(&inputFile);
       while (!in.atEnd())
       {
          line += in.readLine();
       }
       inputFile.close();
    }
//    qDebug()<<line;


    QRegExp rx("([+-]?\\d+)\\.(\\d+)");

    QList<QString> list;
    int pos = 0;

    while ((pos = rx.indexIn(line, pos)) != -1) {
        list << rx.cap(1) + "." + rx.cap(2);
        pos += rx.matchedLength();
    }
    if(list.size() < 3) {
        QMessageBox msgBox;
        msgBox.setWindowTitle("ERROR");
        msgBox.setText("The YAML file is not well configured");
        msgBox.setInformativeText("Please select the propper configuration file.");
        msgBox.exec();
    }
    //set coordinates from list
    if(list.size() >= 3) {
        ui->x1->setValue(list[0].toDouble());
        ui->y1->setValue(list[1].toDouble());
        ui->a1->setValue(list[2].toDouble());
        ui->pioneer1->setChecked(true);
    }
    else {
        ui->x1->setValue(0);
        ui->y1->setValue(0);
        ui->a1->setValue(0);
    }

    if(list.size() >= 6) {
    ui->x2->setValue(list[3].toDouble());
    ui->y2->setValue(list[4].toDouble());
    ui->a2->setValue(list[5].toDouble());
    ui->pioneer2->setChecked(true);
    }
    else {
        ui->x2->setValue(0);
        ui->y2->setValue(0);
        ui->a2->setValue(0);
    }

    if(list.size() >= 9) {
    ui->x3->setValue(list[6].toDouble());
    ui->y3->setValue(list[7].toDouble());
    ui->a3->setValue(list[8].toDouble());
    ui->pioneer3->setChecked(true);
    }
    else {
        ui->x3->setValue(0);
        ui->y3->setValue(0);
        ui->a3->setValue(0);
    }

    if(list.size() >= 12) {
    ui->x4->setValue(list[9].toDouble());
    ui->y4->setValue(list[10].toDouble());
    ui->a4->setValue(list[11].toDouble());
    ui->pioneer4->setChecked(true);
    }
    else {
        ui->x4->setValue(0);
        ui->y4->setValue(0);
        ui->a4->setValue(0);
    }

    if(list.size() >= 15) {
    ui->x5->setValue(list[12].toDouble());
    ui->y5->setValue(list[13].toDouble());
    ui->a5->setValue(list[14].toDouble());
    ui->pioneer5->setChecked(true);
    }
    else {
        ui->x5->setValue(0);
        ui->y5->setValue(0);
        ui->a5->setValue(0);
    }
}
void Pioneer_Gui::on_pushButtonRqtGraphNav_clicked()
{
    rqtGraphProcess = new QProcess(this);
    rqtGraphProcess->startDetached("/bin/bash", QStringList() << rqtGraphScriptPath);
    rqtGraphProcess->waitForFinished();
    rqtGraphProcess->kill();
}
void Pioneer_Gui::on_stop_nav_buton_clicked()
{
    QString stopScriptPath = QDir::homePath() + "/script_pid.txt";
    QFile file(stopScriptPath);
    QFile outputPoseYamlFile(outputPoseYamlPath);

    QString line;
    if(!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "n-am apucat sa fac fisierul" << file.errorString();
    }

    QTextStream in(&file);

    while(!in.atEnd()) {
        line = in.readLine();
    }


    QString comanda = "kill -2 " + line;
    qDebug() << comanda;


    myStopProcess = new QProcess(this);
    myStopProcess->start(comanda);
    myStopProcess->waitForFinished();
    myStopProcess->kill();
    myStartProcess->kill();
    file.close();
    file.remove();
    outputPoseYamlFile.remove();
}

void Pioneer_Gui::on_pushButton_2_clicked()
{
    // How many robots are used
    if(ui->pioneer1->isChecked())
    {
        numberOfUsedRobots = "1";
    }
    else if (ui->pioneer2->isChecked())
    {
        numberOfUsedRobots = "2";
    }
    else if (ui->pioneer3->isChecked())
    {
        numberOfUsedRobots = "3";
    }
    else if (ui->pioneer4->isChecked())
    {
        numberOfUsedRobots = "4";
    }
    else if (ui->pioneer5->isChecked())
    {
        numberOfUsedRobots = "5";
    }

    QFile outputPoseYamlFile(outputPoseYamlPath);
    //create yaml
    YAML::Emitter out;
    out << YAML::BeginMap;
    for(qint8 i = 1; i <= numberOfUsedRobots.toInt(); ++i) {
        QString pioneer_name;
        QString xString;
        QString yString;
        QString aString;
        switch (i) {
            case 1: {
                pioneer_name = "pioneer" + QString::number(i);
                xString = QString::number(ui->x1->value());
                yString = QString::number(ui->y1->value());
                aString = QString::number(ui->a1->value());
                break;
            }
            case 2: {
                pioneer_name = "pioneer" + QString::number(i);
                xString = QString::number(ui->x2->value());
                yString = QString::number(ui->y2->value());
                aString = QString::number(ui->a2->value());
                break;
            }
            case 3: {
                pioneer_name = "pioneer" + QString::number(i);
                xString = QString::number(ui->x3->value());
                yString = QString::number(ui->y3->value());
                aString = QString::number(ui->a3->value());
                break;
            }
            case 4: {
                pioneer_name = "pioneer" + QString::number(i);
                xString = QString::number(ui->x4->value());
                yString = QString::number(ui->y4->value());
                aString = QString::number(ui->a4->value());
                break;
            }
            case 5: {
                pioneer_name = "pioneer" + QString::number(i);
                xString = QString::number(ui->x5->value());
                yString = QString::number(ui->y5->value());
                aString = QString::number(ui->a5->value());
                break;
            }
        }
        out << YAML::Key << pioneer_name.toStdString();
        out << YAML::Value << YAML::BeginMap << YAML::Key << "x" <<YAML::Value << xString.toStdString() << YAML::Key << "y" <<YAML::Value << yString.toStdString() << YAML::Key << "a" <<YAML::Value << aString.toStdString() << YAML::EndMap;
    }
    out << YAML::EndMap;
    if (outputPoseYamlFile.open(QIODevice::ReadWrite))
    {
        QTextStream outputStream( &outputPoseYamlFile );
        outputStream << out.c_str();
    }
}

