#include "real.h"
#include "ui_real.h"
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

Real::Real(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Real)
{
    ui->setupUi(this);
    connect(ui->pushButton_home, SIGNAL(clicked()), this->parent(), SLOT(back_to_main_menu()));

    //  source path
    packagePath = QString::fromStdString(ros::package::getPath("pioneer_gui"));
    worldImagePath = packagePath + "/resources/worlds_jpg/" + ui->comboBox_worldsNavigation->itemText(ui->comboBox_worldsNavigation->currentIndex()) + ".jpg";
    worldImage = new QPixmap();
    worldImage->load(worldImagePath);
    ui->label_imageWorldsNavigation->setScaledContents(true);
    ui->label_imageWorldsNavigation->setPixmap(*worldImage);

    //  Shell scripts paths
    realMappingScriptPath = packagePath + "/resources/shell_scripts/real_mapping.sh";
    realNavigationScriptPath = packagePath + "/resources/shell_scripts/real_navigation.sh";

    // Pose source file Paths
    poseFileDefaultPath = packagePath.left(packagePath.indexOf("pioneer_gui")) + "pioneer_description/params";
    poseFilePathFromMapping = poseFileDefaultPath + "/pioneer_pose_from_mapping.yaml";
    poseFilePathFromNavigation = poseFileDefaultPath + "/pioneer_poses_from_navigation.yaml";

    pidTxtFilePath = (packagePath.left(packagePath.indexOf("/",6) + 1)) + "script_pid.txt";
    usedPoseFilePathForMappig = poseFileDefaultPath + "/created_yaml_pose_for_mapping.yaml";
    usedPoseFilePathForNavigation = poseFileDefaultPath + "/created_yaml_poses_for_navigation.yaml";
    usedUsernameFilePathForMapping = poseFileDefaultPath + "/created_yaml_username_for_mapping.yaml";
    usedUsernameFilePathForNavigation = poseFileDefaultPath + "/created_yaml_username_for_navigation.yaml";
}

Real::~Real()
{
    delete ui;
}

void Real::complete_usernames_from_yaml(const QString& filePath, const QString& module)
{
    QFile inputFile(filePath);
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
    if(line.indexOf(QRegExp( "(pioneer[0-9]-username:)" ))) {
        QMessageBox msgBox;
        msgBox.setWindowTitle("ERROR");
        msgBox.setText("The username YAML file you chose is not well formated.");
        msgBox.exec();
        if(module == "mapping")
            ui->lineEdit_usernameFileMapping->clear();
        else if (module == "navigation") {
            ui->lineEdit_usernameFileNavigation->clear();
        }
        return;
    }

    line.remove( QRegExp( "(pioneer[0-9]-username:)" ) );
//    qDebug() << line;
    QRegExp rx("(pioneer[0-9]-username: )");
    line.remove(0,1);
    QList<QString> list;
    list = line.split(" ");
    qDebug() << list;


    if( module == "mapping" ) {
        ui->comboBox_usernamesMapping->clear();
        for (int i = 0; i < list.length(); ++i) {
            ui->comboBox_usernamesMapping->addItem(list[i]);
        }
        ui->lineEdit_usernameFileMapping->setText(filePath);
    }

    else if (module == "navigation") {
        switch (list.length()) {
            case 1: {
                ui->radioButton_pioneer1->setChecked(true);
                ui->radioButton_pioneer1->setText(list[0]);
                ui->radioButton_pioneer2->setEnabled(false);
                ui->radioButton_pioneer3->setEnabled(false);
                ui->radioButton_pioneer4->setEnabled(false);
                ui->radioButton_pioneer5->setEnabled(false);
                ui->radioButton_pioneer2->setText("2");
                ui->radioButton_pioneer3->setText("3");
                ui->radioButton_pioneer4->setText("4");
                ui->radioButton_pioneer5->setText("5");
                break;
            }
            case 2 : {
                ui->radioButton_pioneer2->setEnabled(true);
                ui->radioButton_pioneer2->setChecked(true);
                ui->radioButton_pioneer1->setText(list[0]);
                ui->radioButton_pioneer2->setText(list[1]);
                ui->radioButton_pioneer3->setEnabled(false);
                ui->radioButton_pioneer4->setEnabled(false);
                ui->radioButton_pioneer5->setEnabled(false);
                ui->radioButton_pioneer3->setText("3");
                ui->radioButton_pioneer4->setText("4");
                ui->radioButton_pioneer5->setText("5");
                break;
            }
            case 3 : {
                ui->radioButton_pioneer2->setEnabled(true);
                ui->radioButton_pioneer3->setEnabled(true);
                ui->radioButton_pioneer3->setChecked(true);
                ui->radioButton_pioneer1->setText(list[0]);
                ui->radioButton_pioneer2->setText(list[1]);
                ui->radioButton_pioneer3->setText(list[2]);
                ui->radioButton_pioneer4->setEnabled(false);
                ui->radioButton_pioneer5->setEnabled(false);
                ui->radioButton_pioneer4->setText("4");
                ui->radioButton_pioneer5->setText("5");
                break;
            }
            case 4 : {
                ui->radioButton_pioneer2->setEnabled(true);
                ui->radioButton_pioneer3->setEnabled(true);
                ui->radioButton_pioneer4->setEnabled(true);
                ui->radioButton_pioneer4->setChecked(true);
                ui->radioButton_pioneer1->setText(list[0]);
                ui->radioButton_pioneer2->setText(list[1]);
                ui->radioButton_pioneer3->setText(list[2]);
                ui->radioButton_pioneer4->setText(list[3]);
                ui->radioButton_pioneer5->setEnabled(false);
                ui->radioButton_pioneer5->setText("5");
                break;
            }
            case 5 : {
                ui->radioButton_pioneer2->setEnabled(true);
                ui->radioButton_pioneer3->setEnabled(true);
                ui->radioButton_pioneer4->setEnabled(true);
                ui->radioButton_pioneer5->setEnabled(true);
                ui->radioButton_pioneer5->setChecked(true);
                ui->radioButton_pioneer1->setText(list[0]);
                ui->radioButton_pioneer2->setText(list[1]);
                ui->radioButton_pioneer3->setText(list[2]);
                ui->radioButton_pioneer4->setText(list[3]);
                ui->radioButton_pioneer5->setText(list[4]);
                break;
            }
        }
        ui->lineEdit_usernameFileNavigation->setText(filePath);
    }

}
void Real::complete_coordinates_from_yaml(const QString& filePath, const QString& module)
{
    QFile inputFile(filePath);
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

    if( list.size() < 3 ) {
        QMessageBox msgBox;
        msgBox.setWindowTitle("ERROR");
        msgBox.setText("The YAML file is not well configured");
        msgBox.setInformativeText("Please select the propper configuration file.");
        msgBox.exec();
        if(module == "mapping")
            ui->lineEdit_poseFileMapping->clear();
        else if (module == "navigation") {
            ui->lineEdit_poseFileNavigation->clear();
        }
        return;
    }
    if( module == "mapping" ) {
        ui->doubleSpinBox_xPoseMapping->setValue(list[0].toDouble());
        ui->doubleSpinBox_yPoseMapping->setValue(list[1].toDouble());
        ui->doubleSpinBox_yawPoseMapping->setValue(list[2].toDouble());
        ui->lineEdit_poseFileMapping->setText(filePath);
    }
    else if( module == "navigation" ) {
        ui->doubleSpinBox_x1PoseNavigation->setValue(list[0].toDouble());
        ui->doubleSpinBox_y1PoseNavigation->setValue(list[1].toDouble());
        ui->doubleSpinBox_yaw1PoseNavigation->setValue(list[2].toDouble());
    if(list.size() >= 6) {
        ui->doubleSpinBox_x2PoseNavigation->setValue(list[3].toDouble());
        ui->doubleSpinBox_y2PoseNavigation->setValue(list[4].toDouble());
        ui->doubleSpinBox_yaw2PoseNavigation->setValue(list[5].toDouble());
        }
    if(list.size() >= 9) {
        ui->doubleSpinBox_x3PoseNavigation->setValue(list[6].toDouble());
        ui->doubleSpinBox_y3PoseNavigation->setValue(list[7].toDouble());
        ui->doubleSpinBox_yaw3PoseNavigation->setValue(list[8].toDouble());
        }
    if(list.size() >= 12) {
        ui->doubleSpinBox_x4PoseNavigation->setValue(list[9].toDouble());
        ui->doubleSpinBox_y4PoseNavigation->setValue(list[10].toDouble());
        ui->doubleSpinBox_yaw4PoseNavigation->setValue(list[11].toDouble());
        }
    if(list.size() >= 15) {
        ui->doubleSpinBox_x5PoseNavigation->setValue(list[12].toDouble());
        ui->doubleSpinBox_y5PoseNavigation->setValue(list[13].toDouble());
        ui->doubleSpinBox_yaw5PoseNavigation->setValue(list[14].toDouble());
        }
    ui->lineEdit_poseFileNavigation->setText(filePath);
    }
}

void Real::create_yaml_from_usernames(const QString& filePath, const QString& module, const int& nrOfUsedRobots)
{
    QFile outputPoseYamlFile(filePath);

    YAML::Emitter out;

    QString pioneer_key;
    QString pioneer_username;

    out << YAML::BeginMap;

    if( module == "mapping" ) {
        pioneer_key = "pioneer1-username";
        pioneer_username = ui->comboBox_usernamesMapping->currentText();
        out << YAML::Key << pioneer_key.toStdString();
        out << YAML::Value << pioneer_username.toStdString();
    }
    else if( module == "navigation" ) {
        for (int i = 1; i <= nrOfUsedRobots; ++i) {
            switch (i) {
                case 1: {
                    pioneer_key = "pioneer1-username";
                    pioneer_username = ui->radioButton_pioneer1->text();
                    break;
                }
                case 2: {
                    pioneer_key = "pioneer2-username";
                    pioneer_username = ui->radioButton_pioneer2->text();
                    break;
                }
                case 3: {
                    pioneer_key = "pioneer3-username";
                    pioneer_username = ui->radioButton_pioneer3->text();
                    break;
                }
                case 4: {
                    pioneer_key = "pioneer4-username";
                pioneer_username = ui->radioButton_pioneer4->text();
                    break;
                }
                case 5: {
                    pioneer_key = "pioneer5-username";
                    pioneer_username = ui->radioButton_pioneer4->text();
                    break;
                }
            }
            out << YAML::Key << pioneer_key.toStdString();
            out << YAML::Value << pioneer_username.toStdString();
        }
    }
    out << YAML::EndMap;

    if (outputPoseYamlFile.open(QIODevice::ReadWrite))
        {
            QTextStream outputStream( &outputPoseYamlFile );
            outputStream << out.c_str();
    }
}

void Real::create_yaml_from_coordinates(const QString& filePath, const QString& module, const int& nrOfUsedRobots)
{
    QFile outputPoseYamlFile(filePath);

    YAML::Emitter out;

    QString pioneer_name;
    QString xString;
    QString yString;
    QString aString;

    out << YAML::BeginMap;

    if( module == "mapping" ) {
        pioneer_name = "pioneer1";
        xString = QString::number(ui->doubleSpinBox_xPoseMapping->value());
        yString = QString::number(ui->doubleSpinBox_yPoseMapping->value());
        aString = QString::number(ui->doubleSpinBox_yawPoseMapping->value());
        out << YAML::Key << pioneer_name.toStdString();
        out << YAML::Value << YAML::BeginMap << YAML::Key << "x" <<YAML::Value << xString.toStdString() <<
               YAML::Key << "y" <<YAML::Value << yString.toStdString() << YAML::Key << "a" <<YAML::Value <<
               aString.toStdString() << YAML::EndMap;
    }
    else if( module == "navigation" ) {
        for (int i = 1; i <= nrOfUsedRobots; ++i) {
            switch (i) {
                case 1: {
                    pioneer_name = "pioneer" + QString::number(i);
                    xString = QString::number(ui->doubleSpinBox_x1PoseNavigation->value());
                    yString = QString::number(ui->doubleSpinBox_y1PoseNavigation->value());
                    aString = QString::number(ui->doubleSpinBox_yaw1PoseNavigation->value());
                    break;
                }
                case 2: {
                    pioneer_name = "pioneer" + QString::number(i);
                    xString = QString::number(ui->doubleSpinBox_x2PoseNavigation->value());
                    yString = QString::number(ui->doubleSpinBox_y2PoseNavigation->value());
                    aString = QString::number(ui->doubleSpinBox_yaw2PoseNavigation->value());
                    break;
                }
                case 3: {
                    pioneer_name = "pioneer" + QString::number(i);
                    xString = QString::number(ui->doubleSpinBox_x3PoseNavigation->value());
                    yString = QString::number(ui->doubleSpinBox_y3PoseNavigation->value());
                    aString = QString::number(ui->doubleSpinBox_yaw3PoseNavigation->value());
                    break;
                }
                case 4: {
                    pioneer_name = "pioneer" + QString::number(i);
                    xString = QString::number(ui->doubleSpinBox_x4PoseNavigation->value());
                    yString = QString::number(ui->doubleSpinBox_y4PoseNavigation->value());
                    aString = QString::number(ui->doubleSpinBox_yaw4PoseNavigation->value());
                    break;
                }
                case 5: {
                    pioneer_name = "pioneer" + QString::number(i);
                    xString = QString::number(ui->doubleSpinBox_x5PoseNavigation->value());
                    yString = QString::number(ui->doubleSpinBox_y5PoseNavigation->value());
                    aString = QString::number(ui->doubleSpinBox_yaw5PoseNavigation->value());
                    break;
                }
            }
            out << YAML::Key << pioneer_name.toStdString();
            out << YAML::Value << YAML::BeginMap << YAML::Key << "x" <<YAML::Value << xString.toStdString() <<
                   YAML::Key << "y" <<YAML::Value << yString.toStdString() << YAML::Key << "a" <<YAML::Value <<
                   aString.toStdString() << YAML::EndMap;
        }
    }
    out << YAML::EndMap;

    if (outputPoseYamlFile.open(QIODevice::ReadWrite))
        {
            QTextStream outputStream( &outputPoseYamlFile );
            outputStream << out.c_str();
    }
}
void Real::findAndDestroy(QProcess *startedProcess, const QString& createdYamlFilePath)
{
    QFile pidTxtFile(pidTxtFilePath);
    QFile createdYamlFile(createdYamlFilePath);
    qDebug() << createdYamlFilePath;
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

    godKillerProcess = new QProcess(this);
    godKillerProcess->start(killPidsCommand);
    godKillerProcess->waitForFinished();
    godKillerProcess->kill();
    startedProcess->kill();
    pidTxtFile.close();
    pidTxtFile.remove();
    createdYamlFile.remove();
}

void Real::oneShotProcess(const QString& package, const QString& node, const QString& optionalArgument1, const QString& optionalArgument2)
{
    if( optionalArgument1 != "" ) {
        if (optionalArgument2 != "") {
            godKillerProcess = new QProcess(this);
            godKillerProcess->startDetached("/bin/bash", QStringList() << "rosrun" << package << node << optionalArgument1 << optionalArgument2);
            godKillerProcess->waitForFinished();
            godKillerProcess->kill();
        }
        godKillerProcess = new QProcess(this);
        godKillerProcess->startDetached("/bin/bash", QStringList() << "rosrun" << package << node << optionalArgument1);
        godKillerProcess->waitForFinished();
        godKillerProcess->kill();
    }
    else {
        godKillerProcess = new QProcess(this);
        godKillerProcess->startDetached("/bin/bash", QStringList() << "rosrun" << package << node);
        godKillerProcess->waitForFinished();
        godKillerProcess->kill();
    }
}
/*  Mapping Functions   */
void Real::on_doubleSpinBox_yawPoseMapping_valueChanged(double arg1)
{
    int dialValue = int(arg1 * 100);
    ui->dial_yawPoseMapping->setValue(dialValue);
}

void Real::on_dial_yawPoseMapping_valueChanged(int value)
{
    double doubleSpinBoxValue = value / 100.0;
    ui->doubleSpinBox_yawPoseMapping->setValue(doubleSpinBoxValue);
}

void Real::on_pushButton_openPoseFileMapping_clicked()
{
    QString poseFileSourcePath = QFileDialog::getOpenFileName(this,
        tr("Open YAML file"), poseFileDefaultPath, tr("YAML Files (*.yaml)"));
//    qDebug() << poseFileSourcePath;

    complete_coordinates_from_yaml(poseFileSourcePath, "mapping");
}

void Real::on_lineEdit_poseFileMapping_returnPressed()
{
    complete_coordinates_from_yaml(ui->lineEdit_poseFileMapping->text(), "mapping");
}

void Real::on_pushButton_startMappingTool_clicked()
{
    create_yaml_from_coordinates(usedPoseFilePathForMappig, "mapping");
    create_yaml_from_usernames(usedUsernameFilePathForMapping, "mapping");
    robot_names_fileArgument = "created_yaml_username_for_mapping";
    pose_fileArgument = "created_yaml_pose_for_mapping";
    gmapping_config_typeArgument = "gmapping_kinect";

    realMappingScriptCommand = realMappingScriptPath + " --pose_file " + pose_fileArgument;
    realMappingScriptCommand += " --robot_names_file " + robot_names_fileArgument;
    realMappingScriptCommand += " --gmapping_config_type " + gmapping_config_typeArgument;
    qDebug() << realMappingScriptCommand;

//    moduleStartProcess = new QProcess(this);
//    moduleStartProcess->startDetached("/bin/bash", QStringList() << realMappingScriptPath << "--pose_file" << pose_fileArgument <<
//                                      "--robot_names_file" << robot_names_fileArgument <<
//                                      "--gmapping_config_type" << gmapping_config_typeArgument);
}

void Real::on_pushButton_openTeleopMapping_clicked()
{
    oneShotProcess("pioneer_key_teleop", "pioneer_key_teleop_node");
}

void Real::on_pushButton_openRqtGraphMapping_clicked()
{
    oneShotProcess("rqt_graph","rqt_graph");
}

void Real::on_pushButton_saveMap_clicked()
{
    if( ui->lineEdit_CreatedMapName->text().isEmpty() ) {
        QMessageBox msgBox;
        msgBox.setWindowTitle("ERROR");
        msgBox.setText("You did not choose a name for the created map!");
        msgBox.setInformativeText("Please set the name in the corresponding textbox.");
        msgBox.exec();
    }

    QString createdMapFileArgument = packagePath.left(packagePath.indexOf("pioneer_gui")) + "pioneer_gazebo/map/" + ui->lineEdit_CreatedMapName->text();

    oneShotProcess("map_server", "map_saver", "-f", createdMapFileArgument);
}

void Real::on_pushButton_stopMappingTool_clicked()
{
    findAndDestroy(moduleStartProcess, usedPoseFilePathForMappig + "");
}


/*  Navigation Fuctions*/
void Real::on_comboBox_worldsNavigation_currentIndexChanged(const QString &arg1)
{
    worldImagePath = packagePath + "/resources/worlds_jpg/" + arg1 + ".jpg";
    delete worldImage;
    worldImage = new QPixmap();
    worldImage->load(worldImagePath);
    ui->label_imageWorldsNavigation->setScaledContents(true);
    ui->label_imageWorldsNavigation->setPixmap(*worldImage);
}

void Real::on_doubleSpinBox_yaw1PoseNavigation_valueChanged(double arg1)
{
    int dialValue = int(arg1 * 100);
    ui->dial_yaw1PoseNavigation->setValue(dialValue);
}

void Real::on_dial_yaw1PoseNavigation_valueChanged(int value)
{
    double doubleSpinBoxValue = value / 100.0;
    ui->doubleSpinBox_yaw1PoseNavigation->setValue(doubleSpinBoxValue);
}

void Real::on_doubleSpinBox_yaw2PoseNavigation_valueChanged(double arg1)
{
    int dialValue = int(arg1 * 100);
    ui->dial_yaw2PoseNavigation->setValue(dialValue);
}

void Real::on_dial_yaw2PoseNavigation_valueChanged(int value)
{
    double doubleSpinBoxValue = value / 100.0;
    ui->doubleSpinBox_yaw2PoseNavigation->setValue(doubleSpinBoxValue);
}

void Real::on_doubleSpinBox_yaw3PoseNavigation_valueChanged(double arg1)
{
    int dialValue = int(arg1 * 100);
    ui->dial_yaw3PoseNavigation->setValue(dialValue);
}

void Real::on_dial_yaw3PoseNavigation_valueChanged(int value)
{
    double doubleSpinBoxValue = value / 100.0;
    ui->doubleSpinBox_yaw3PoseNavigation->setValue(doubleSpinBoxValue);
}

void Real::on_doubleSpinBox_yaw4PoseNavigation_valueChanged(double arg1)
{
    int dialValue = int(arg1 * 100);
    ui->dial_yaw4PoseNavigation->setValue(dialValue);
}

void Real::on_dial_yaw4PoseNavigation_valueChanged(int value)
{
    double doubleSpinBoxValue = value / 100.0;
    ui->doubleSpinBox_yaw4PoseNavigation->setValue(doubleSpinBoxValue);
}

void Real::on_doubleSpinBox_yaw5PoseNavigation_valueChanged(double arg1)
{
    int dialValue = int(arg1 * 100);
    ui->dial_yaw5PoseNavigation->setValue(dialValue);
}

void Real::on_dial_yaw5PoseNavigation_valueChanged(int value)
{
    double doubleSpinBoxValue = value / 100.0;
    ui->doubleSpinBox_yaw5PoseNavigation->setValue(doubleSpinBoxValue);
}

void Real::on_pushButton_openPoseFileNavigation_clicked()
{
    QString poseFileSourcePath = QFileDialog::getOpenFileName(this,
        tr("Open YAML file"), poseFileDefaultPath, tr("YAML Files (*.yaml)"));
//    qDebug() << poseFileSourcePath;

    complete_coordinates_from_yaml(poseFileSourcePath, "navigation");
}

void Real::on_lineEdit_poseFileNavigation_returnPressed()
{
    complete_coordinates_from_yaml(ui->lineEdit_poseFileNavigation->text(),"navigation");
}

void Real::on_pushButton_startNavigationTool_clicked()
{
    worldArgument = ui->comboBox_worldsNavigation->currentText();

    robot_URDF_modelArgument = "pioneer_kinect";
    kinect_to_laserscanArgument = "true";

    // How many robots are used
        if(ui->radioButton_pioneer1->isChecked())
        {
            numberOfUsedRobots = 1;
        }
        else if (ui->radioButton_pioneer2->isChecked())
        {
            numberOfUsedRobots = 2;
        }
        else if (ui->radioButton_pioneer3->isChecked())
        {
            numberOfUsedRobots = 3;
        }
        else if (ui->radioButton_pioneer4->isChecked())
        {
            numberOfUsedRobots = 4;
        }
        else if (ui->radioButton_pioneer5->isChecked())
        {
            numberOfUsedRobots = 5;
    }

    create_yaml_from_coordinates(usedPoseFilePathForNavigation, "navigation", numberOfUsedRobots);
    create_yaml_from_usernames(usedUsernameFilePathForNavigation, "navigation", numberOfUsedRobots);
    robot_names_fileArgument = "created_yaml_username_for_navigation";
    pose_fileArgument = "created_yaml_poses_for_navigation";

    localization_typeArgument = ui->comboBox_localizationMethod->currentText();
    base_global_plannerArgument = ui->comboBox_baseGlobalPlanner->currentText();
    base_local_plannerArgument = ui->comboBox_baseLocalPlanner->currentText();
    rviz_configArgument = QString::number(numberOfUsedRobots) + "_robots_navigation_" + base_global_plannerArgument + "_" + base_local_plannerArgument;

    realNavigationScriptCommand = realNavigationScriptPath + " --pose_file " + pose_fileArgument;
    realNavigationScriptCommand += " --map_name " + ui->comboBox_worldsNavigation->currentText();
    realNavigationScriptCommand += " --robot_names_file " + robot_names_fileArgument;    // check radiobutton and create yaml.
    realNavigationScriptCommand += " --participants " + QString::number(numberOfUsedRobots);
    realNavigationScriptCommand += " --localization_type " + localization_typeArgument;
    realNavigationScriptCommand += " --base_global_planner " + base_global_plannerArgument;
    realNavigationScriptCommand += " --base_local_planner " + base_local_plannerArgument;
    realNavigationScriptCommand += "--rviz_config " + rviz_configArgument;
    qDebug() << realNavigationScriptCommand;

//    moduleStartProcess = new QProcess(this);
//    moduleStartProcess->startDetached("/bin/bash", QStringList() << realNavigationScriptPath <<
//                                      "--pose_file" << pose_fileArgument <<
//                                      "--map_name" << ui->comboBox_worldsNavigation->currentText() <<
//                                      "--robot_names_file" << robot_names_fileArgument <<
//                                      "--participants" << QString::number(numberOfUsedRobots) <<
//                                      "--localization_type" << localization_typeArgument <<
//                                      "--base_global_planner" <<base_global_plannerArgument <<
//                                      "--base_local_planner" << base_local_plannerArgument <<
//                                      "--rviz_config" << rviz_configArgument);
}

void Real::on_pushButton_openRqtGraphNav_clicked()
{
    oneShotProcess("rqt_graph","rqt_graph");
}
void Real::on_pushButton_openRqtReconfigureNav_clicked()
{
    oneShotProcess("rqt_reconfigure", "rqt_reconfigure");
}

void Real::on_pushButton_stopNavigationTool_clicked()
{
    findAndDestroy(moduleStartProcess, usedPoseFilePathForNavigation);
}


void Real::on_pushButton_openUsernameFileMapping_clicked()
{
    QString usernameFileSourcePath = QFileDialog::getOpenFileName(this,
        tr("Open YAML file"), poseFileDefaultPath, tr("YAML Files (*.yaml)"));
//    qDebug() << poseFileSourcePath;

    complete_usernames_from_yaml(usernameFileSourcePath, "mapping");
}

void Real::on_lineEdit_usernameFileMapping_returnPressed()
{
    QString usernameFileSourcePathFromLine = ui->lineEdit_usernameFileMapping->text();
    complete_usernames_from_yaml(usernameFileSourcePathFromLine, "mapping");
}

void Real::on_pushButton_openUsernameFileNavigation_clicked()
{
    QString usernameFileSourcePath = QFileDialog::getOpenFileName(this,
        tr("Open YAML file"), poseFileDefaultPath, tr("YAML Files (*.yaml)"));
//    qDebug() << poseFileSourcePath;

    complete_usernames_from_yaml(usernameFileSourcePath, "navigation");

}
