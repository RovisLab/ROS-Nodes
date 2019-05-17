/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QTabWidget *tabWidget;
    QWidget *tab;
    QGroupBox *groupBox;
    QComboBox *combo_box_mapping;
    QLabel *img_mapping;
    QGroupBox *groupBox_2;
    QRadioButton *hokuyo_gmapping;
    QRadioButton *kinect_gmapping;
    QGroupBox *groupBox_3;
    QLabel *label_26;
    QLineEdit *lineEditCreatedMapName;
    QPushButton *save_map;
    QPushButton *stopButon;
    QPushButton *start_mapping;
    QPushButton *pushButtonRqtGraph;
    QPushButton *start_teleop;
    QWidget *tab_2;
    QGroupBox *groupBox_4;
    QComboBox *combo_box_nav;
    QLabel *img_nav;
    QGroupBox *groupBox_5;
    QGridLayout *gridLayout_3;
    QRadioButton *hokuyo_nav;
    QRadioButton *kinect_nav;
    QGroupBox *groupBox_6;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout_2;
    QRadioButton *pioneer1;
    QRadioButton *pioneer2;
    QRadioButton *pioneer3;
    QRadioButton *pioneer4;
    QRadioButton *pioneer5;
    QLabel *label_17;
    QLabel *label_18;
    QDoubleSpinBox *x1;
    QDoubleSpinBox *y1;
    QLabel *label_19;
    QDoubleSpinBox *x2;
    QDoubleSpinBox *y2;
    QLabel *label_20;
    QLabel *label_21;
    QDoubleSpinBox *x3;
    QDoubleSpinBox *y3;
    QLabel *label_22;
    QLabel *label_23;
    QDoubleSpinBox *x4;
    QDoubleSpinBox *y4;
    QLabel *label_37;
    QLabel *label_38;
    QLabel *label_39;
    QDoubleSpinBox *y5;
    QDoubleSpinBox *x5;
    QPushButton *pushButton;
    QLabel *label_2;
    QLineEdit *lineEditPoseFile;
    QDoubleSpinBox *a1;
    QLabel *label_24;
    QLabel *label_40;
    QDoubleSpinBox *a5;
    QDoubleSpinBox *a3;
    QDoubleSpinBox *a4;
    QLabel *label_25;
    QLabel *label_41;
    QLabel *label_27;
    QDoubleSpinBox *a2;
    QPushButton *pushButtonRqtGraphNav;
    QPushButton *start_nav;
    QPushButton *stop_nav_buton;
    QGroupBox *groupBox_7;
    QComboBox *comboBox_local_planner;
    QLabel *label;
    QComboBox *comboBox_global_planner;
    QLabel *nav_global_planner_label;
    QLabel *nav_local_planner_label;
    QComboBox *comboBox_localization_type;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(894, 760);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        groupBox = new QGroupBox(tab);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(9, 9, 854, 207));
        groupBox->setFlat(false);
        combo_box_mapping = new QComboBox(groupBox);
        combo_box_mapping->setObjectName(QStringLiteral("combo_box_mapping"));
        combo_box_mapping->setGeometry(QRect(30, 90, 101, 27));
        img_mapping = new QLabel(groupBox);
        img_mapping->setObjectName(QStringLiteral("img_mapping"));
        img_mapping->setGeometry(QRect(280, 30, 311, 151));
        groupBox_2 = new QGroupBox(tab);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(9, 222, 854, 71));
        hokuyo_gmapping = new QRadioButton(groupBox_2);
        hokuyo_gmapping->setObjectName(QStringLiteral("hokuyo_gmapping"));
        hokuyo_gmapping->setGeometry(QRect(10, 30, 117, 22));
        kinect_gmapping = new QRadioButton(groupBox_2);
        kinect_gmapping->setObjectName(QStringLiteral("kinect_gmapping"));
        kinect_gmapping->setGeometry(QRect(150, 30, 141, 22));
        groupBox_3 = new QGroupBox(tab);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(10, 350, 854, 101));
        label_26 = new QLabel(groupBox_3);
        label_26->setObjectName(QStringLiteral("label_26"));
        label_26->setGeometry(QRect(20, 40, 231, 17));
        lineEditCreatedMapName = new QLineEdit(groupBox_3);
        lineEditCreatedMapName->setObjectName(QStringLiteral("lineEditCreatedMapName"));
        lineEditCreatedMapName->setGeometry(QRect(250, 30, 113, 27));
        save_map = new QPushButton(groupBox_3);
        save_map->setObjectName(QStringLiteral("save_map"));
        save_map->setGeometry(QRect(20, 60, 141, 27));
        stopButon = new QPushButton(tab);
        stopButon->setObjectName(QStringLiteral("stopButon"));
        stopButon->setGeometry(QRect(220, 600, 99, 27));
        start_mapping = new QPushButton(tab);
        start_mapping->setObjectName(QStringLiteral("start_mapping"));
        start_mapping->setGeometry(QRect(10, 600, 181, 27));
        pushButtonRqtGraph = new QPushButton(tab);
        pushButtonRqtGraph->setObjectName(QStringLiteral("pushButtonRqtGraph"));
        pushButtonRqtGraph->setGeometry(QRect(730, 600, 141, 31));
        start_teleop = new QPushButton(tab);
        start_teleop->setObjectName(QStringLiteral("start_teleop"));
        start_teleop->setGeometry(QRect(680, 560, 191, 27));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        groupBox_4 = new QGroupBox(tab_2);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        groupBox_4->setGeometry(QRect(9, 9, 771, 141));
        groupBox_4->setFlat(false);
        combo_box_nav = new QComboBox(groupBox_4);
        combo_box_nav->setObjectName(QStringLiteral("combo_box_nav"));
        combo_box_nav->setGeometry(QRect(10, 50, 221, 25));
        img_nav = new QLabel(groupBox_4);
        img_nav->setObjectName(QStringLiteral("img_nav"));
        img_nav->setGeometry(QRect(290, 30, 181, 101));
        groupBox_5 = new QGroupBox(tab_2);
        groupBox_5->setObjectName(QStringLiteral("groupBox_5"));
        groupBox_5->setGeometry(QRect(10, 170, 491, 71));
        gridLayout_3 = new QGridLayout(groupBox_5);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        hokuyo_nav = new QRadioButton(groupBox_5);
        hokuyo_nav->setObjectName(QStringLiteral("hokuyo_nav"));

        gridLayout_3->addWidget(hokuyo_nav, 0, 0, 1, 1);

        kinect_nav = new QRadioButton(groupBox_5);
        kinect_nav->setObjectName(QStringLiteral("kinect_nav"));

        gridLayout_3->addWidget(kinect_nav, 0, 1, 1, 1);

        groupBox_6 = new QGroupBox(tab_2);
        groupBox_6->setObjectName(QStringLiteral("groupBox_6"));
        groupBox_6->setGeometry(QRect(0, 270, 854, 161));
        layoutWidget = new QWidget(groupBox_6);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(30, 30, 491, 25));
        horizontalLayout_2 = new QHBoxLayout(layoutWidget);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        pioneer1 = new QRadioButton(layoutWidget);
        pioneer1->setObjectName(QStringLiteral("pioneer1"));

        horizontalLayout_2->addWidget(pioneer1);

        pioneer2 = new QRadioButton(layoutWidget);
        pioneer2->setObjectName(QStringLiteral("pioneer2"));

        horizontalLayout_2->addWidget(pioneer2);

        pioneer3 = new QRadioButton(layoutWidget);
        pioneer3->setObjectName(QStringLiteral("pioneer3"));

        horizontalLayout_2->addWidget(pioneer3);

        pioneer4 = new QRadioButton(layoutWidget);
        pioneer4->setObjectName(QStringLiteral("pioneer4"));

        horizontalLayout_2->addWidget(pioneer4);

        pioneer5 = new QRadioButton(layoutWidget);
        pioneer5->setObjectName(QStringLiteral("pioneer5"));

        horizontalLayout_2->addWidget(pioneer5);

        label_17 = new QLabel(groupBox_6);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setGeometry(QRect(20, 70, 16, 17));
        label_18 = new QLabel(groupBox_6);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setGeometry(QRect(20, 100, 16, 17));
        x1 = new QDoubleSpinBox(groupBox_6);
        x1->setObjectName(QStringLiteral("x1"));
        x1->setGeometry(QRect(40, 60, 69, 27));
        x1->setMinimum(-50);
        y1 = new QDoubleSpinBox(groupBox_6);
        y1->setObjectName(QStringLiteral("y1"));
        y1->setGeometry(QRect(40, 90, 69, 27));
        y1->setMinimum(-50);
        label_19 = new QLabel(groupBox_6);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setGeometry(QRect(120, 70, 16, 17));
        x2 = new QDoubleSpinBox(groupBox_6);
        x2->setObjectName(QStringLiteral("x2"));
        x2->setGeometry(QRect(140, 60, 69, 27));
        x2->setMinimum(-50);
        y2 = new QDoubleSpinBox(groupBox_6);
        y2->setObjectName(QStringLiteral("y2"));
        y2->setGeometry(QRect(140, 90, 69, 27));
        y2->setMinimum(-50);
        label_20 = new QLabel(groupBox_6);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setGeometry(QRect(120, 100, 16, 17));
        label_21 = new QLabel(groupBox_6);
        label_21->setObjectName(QStringLiteral("label_21"));
        label_21->setGeometry(QRect(220, 70, 16, 17));
        x3 = new QDoubleSpinBox(groupBox_6);
        x3->setObjectName(QStringLiteral("x3"));
        x3->setGeometry(QRect(240, 60, 69, 27));
        x3->setMinimum(-50);
        y3 = new QDoubleSpinBox(groupBox_6);
        y3->setObjectName(QStringLiteral("y3"));
        y3->setGeometry(QRect(240, 90, 69, 27));
        y3->setMinimum(-50);
        label_22 = new QLabel(groupBox_6);
        label_22->setObjectName(QStringLiteral("label_22"));
        label_22->setGeometry(QRect(220, 100, 16, 17));
        label_23 = new QLabel(groupBox_6);
        label_23->setObjectName(QStringLiteral("label_23"));
        label_23->setGeometry(QRect(330, 70, 16, 17));
        x4 = new QDoubleSpinBox(groupBox_6);
        x4->setObjectName(QStringLiteral("x4"));
        x4->setGeometry(QRect(350, 60, 69, 27));
        x4->setMinimum(-50);
        y4 = new QDoubleSpinBox(groupBox_6);
        y4->setObjectName(QStringLiteral("y4"));
        y4->setGeometry(QRect(350, 90, 69, 27));
        y4->setMinimum(-50);
        label_37 = new QLabel(groupBox_6);
        label_37->setObjectName(QStringLiteral("label_37"));
        label_37->setGeometry(QRect(330, 100, 16, 17));
        label_38 = new QLabel(groupBox_6);
        label_38->setObjectName(QStringLiteral("label_38"));
        label_38->setGeometry(QRect(430, 70, 16, 17));
        label_39 = new QLabel(groupBox_6);
        label_39->setObjectName(QStringLiteral("label_39"));
        label_39->setGeometry(QRect(430, 100, 16, 17));
        y5 = new QDoubleSpinBox(groupBox_6);
        y5->setObjectName(QStringLiteral("y5"));
        y5->setGeometry(QRect(450, 90, 69, 27));
        y5->setMinimum(-50);
        x5 = new QDoubleSpinBox(groupBox_6);
        x5->setObjectName(QStringLiteral("x5"));
        x5->setGeometry(QRect(450, 60, 69, 27));
        x5->setMinimum(-50);
        pushButton = new QPushButton(groupBox_6);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(570, 60, 161, 21));
        label_2 = new QLabel(groupBox_6);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(570, 40, 221, 20));
        lineEditPoseFile = new QLineEdit(groupBox_6);
        lineEditPoseFile->setObjectName(QStringLiteral("lineEditPoseFile"));
        lineEditPoseFile->setGeometry(QRect(570, 90, 201, 25));
        a1 = new QDoubleSpinBox(groupBox_6);
        a1->setObjectName(QStringLiteral("a1"));
        a1->setGeometry(QRect(40, 120, 69, 27));
        a1->setMinimum(-50);
        label_24 = new QLabel(groupBox_6);
        label_24->setObjectName(QStringLiteral("label_24"));
        label_24->setGeometry(QRect(120, 130, 16, 17));
        label_40 = new QLabel(groupBox_6);
        label_40->setObjectName(QStringLiteral("label_40"));
        label_40->setGeometry(QRect(430, 130, 16, 17));
        a5 = new QDoubleSpinBox(groupBox_6);
        a5->setObjectName(QStringLiteral("a5"));
        a5->setGeometry(QRect(450, 120, 69, 27));
        a5->setMinimum(-50);
        a3 = new QDoubleSpinBox(groupBox_6);
        a3->setObjectName(QStringLiteral("a3"));
        a3->setGeometry(QRect(240, 120, 69, 27));
        a3->setMinimum(-50);
        a4 = new QDoubleSpinBox(groupBox_6);
        a4->setObjectName(QStringLiteral("a4"));
        a4->setGeometry(QRect(350, 120, 69, 27));
        a4->setMinimum(-50);
        label_25 = new QLabel(groupBox_6);
        label_25->setObjectName(QStringLiteral("label_25"));
        label_25->setGeometry(QRect(20, 130, 16, 17));
        label_41 = new QLabel(groupBox_6);
        label_41->setObjectName(QStringLiteral("label_41"));
        label_41->setGeometry(QRect(330, 130, 16, 17));
        label_27 = new QLabel(groupBox_6);
        label_27->setObjectName(QStringLiteral("label_27"));
        label_27->setGeometry(QRect(220, 130, 16, 17));
        a2 = new QDoubleSpinBox(groupBox_6);
        a2->setObjectName(QStringLiteral("a2"));
        a2->setGeometry(QRect(140, 120, 69, 27));
        a2->setMinimum(-50);
        pushButtonRqtGraphNav = new QPushButton(tab_2);
        pushButtonRqtGraphNav->setObjectName(QStringLiteral("pushButtonRqtGraphNav"));
        pushButtonRqtGraphNav->setGeometry(QRect(740, 610, 121, 25));
        start_nav = new QPushButton(tab_2);
        start_nav->setObjectName(QStringLiteral("start_nav"));
        start_nav->setGeometry(QRect(20, 600, 151, 27));
        stop_nav_buton = new QPushButton(tab_2);
        stop_nav_buton->setObjectName(QStringLiteral("stop_nav_buton"));
        stop_nav_buton->setGeometry(QRect(180, 600, 141, 31));
        groupBox_7 = new QGroupBox(tab_2);
        groupBox_7->setObjectName(QStringLiteral("groupBox_7"));
        groupBox_7->setGeometry(QRect(0, 440, 841, 91));
        comboBox_local_planner = new QComboBox(groupBox_7);
        comboBox_local_planner->setObjectName(QStringLiteral("comboBox_local_planner"));
        comboBox_local_planner->setGeometry(QRect(520, 50, 211, 25));
        label = new QLabel(groupBox_7);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(20, 30, 151, 17));
        comboBox_global_planner = new QComboBox(groupBox_7);
        comboBox_global_planner->setObjectName(QStringLiteral("comboBox_global_planner"));
        comboBox_global_planner->setGeometry(QRect(260, 50, 191, 25));
        nav_global_planner_label = new QLabel(groupBox_7);
        nav_global_planner_label->setObjectName(QStringLiteral("nav_global_planner_label"));
        nav_global_planner_label->setGeometry(QRect(260, 30, 221, 17));
        nav_local_planner_label = new QLabel(groupBox_7);
        nav_local_planner_label->setObjectName(QStringLiteral("nav_local_planner_label"));
        nav_local_planner_label->setGeometry(QRect(520, 30, 221, 17));
        comboBox_localization_type = new QComboBox(groupBox_7);
        comboBox_localization_type->setObjectName(QStringLiteral("comboBox_localization_type"));
        comboBox_localization_type->setGeometry(QRect(20, 50, 86, 25));
        tabWidget->addTab(tab_2, QString());

        gridLayout->addWidget(tabWidget, 0, 1, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 894, 22));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Simulation Tool", 0));
        groupBox->setTitle(QApplication::translate("MainWindow", "Choose your world:", 0));
        combo_box_mapping->clear();
        combo_box_mapping->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "maze_one", 0)
         << QApplication::translate("MainWindow", "maze_two", 0)
         << QApplication::translate("MainWindow", "maze_three", 0)
        );
        img_mapping->setText(QApplication::translate("MainWindow", "IMG_M", 0));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "Select the type of laser aquisition device:", 0));
        hokuyo_gmapping->setText(QApplication::translate("MainWindow", "Hokuyo laser", 0));
        kinect_gmapping->setText(QApplication::translate("MainWindow", "Kinect camera", 0));
        groupBox_3->setTitle(QApplication::translate("MainWindow", "Save map:", 0));
        label_26->setText(QApplication::translate("MainWindow", "Set the name of the created map:", 0));
        lineEditCreatedMapName->setText(QString());
        save_map->setText(QApplication::translate("MainWindow", "Save created map", 0));
        stopButon->setText(QApplication::translate("MainWindow", "Stop map", 0));
        start_mapping->setText(QApplication::translate("MainWindow", "Start Mappinig", 0));
        pushButtonRqtGraph->setText(QApplication::translate("MainWindow", "Open rqt_graph", 0));
        start_teleop->setText(QApplication::translate("MainWindow", "Start teleoperation Node", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "Mapping", 0));
        groupBox_4->setTitle(QApplication::translate("MainWindow", "Choose your world:", 0));
        combo_box_nav->clear();
        combo_box_nav->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "maze_one", 0)
         << QApplication::translate("MainWindow", "maze_two", 0)
         << QApplication::translate("MainWindow", "maze_three", 0)
        );
        img_nav->setText(QApplication::translate("MainWindow", "IMG_N", 0));
        groupBox_5->setTitle(QApplication::translate("MainWindow", "Select the type of laser aquisition device:", 0));
        hokuyo_nav->setText(QApplication::translate("MainWindow", "Hokuyo laser", 0));
        kinect_nav->setText(QApplication::translate("MainWindow", "Kinect camera", 0));
        groupBox_6->setTitle(QApplication::translate("MainWindow", "Set the number of robots to be spawned and their initial position:", 0));
        pioneer1->setText(QApplication::translate("MainWindow", "1", 0));
        pioneer2->setText(QApplication::translate("MainWindow", "2", 0));
        pioneer3->setText(QApplication::translate("MainWindow", "3", 0));
        pioneer4->setText(QApplication::translate("MainWindow", "4", 0));
        pioneer5->setText(QApplication::translate("MainWindow", "5", 0));
        label_17->setText(QApplication::translate("MainWindow", "x:", 0));
        label_18->setText(QApplication::translate("MainWindow", "y:", 0));
        label_19->setText(QApplication::translate("MainWindow", "x:", 0));
        label_20->setText(QApplication::translate("MainWindow", "y:", 0));
        label_21->setText(QApplication::translate("MainWindow", "x:", 0));
        label_22->setText(QApplication::translate("MainWindow", "y:", 0));
        label_23->setText(QApplication::translate("MainWindow", "x:", 0));
        label_37->setText(QApplication::translate("MainWindow", "y:", 0));
        label_38->setText(QApplication::translate("MainWindow", "x:", 0));
        label_39->setText(QApplication::translate("MainWindow", "y:", 0));
        pushButton->setText(QApplication::translate("MainWindow", "Open pose file..", 0));
        label_2->setText(QApplication::translate("MainWindow", "Or select predefined pose file:", 0));
        label_24->setText(QApplication::translate("MainWindow", "a:", 0));
        label_40->setText(QApplication::translate("MainWindow", "a:", 0));
        label_25->setText(QApplication::translate("MainWindow", "a:", 0));
        label_41->setText(QApplication::translate("MainWindow", "a:", 0));
        label_27->setText(QApplication::translate("MainWindow", "a:", 0));
        pushButtonRqtGraphNav->setText(QApplication::translate("MainWindow", "Open rqt_graph", 0));
        start_nav->setText(QApplication::translate("MainWindow", "Start Navigation", 0));
        stop_nav_buton->setText(QApplication::translate("MainWindow", "Stop Navigation", 0));
        groupBox_7->setTitle(QApplication::translate("MainWindow", "Select navigation prefferences:", 0));
        comboBox_local_planner->clear();
        comboBox_local_planner->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "DWALocalPlanner", 0)
         << QApplication::translate("MainWindow", "TrajectoryPlannerROS", 0)
         << QApplication::translate("MainWindow", "EBandLocalPlanner", 0)
         << QApplication::translate("MainWindow", "TebLocalPlannerROS", 0)
        );
        label->setText(QApplication::translate("MainWindow", "localization method:", 0));
        comboBox_global_planner->clear();
        comboBox_global_planner->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "NavfnROS", 0)
         << QApplication::translate("MainWindow", "GlobalPlanner", 0)
         << QApplication::translate("MainWindow", "CarrotPlanner", 0)
        );
        nav_global_planner_label->setText(QApplication::translate("MainWindow", "base global planner:", 0));
        nav_local_planner_label->setText(QApplication::translate("MainWindow", "base local planner:", 0));
        comboBox_localization_type->clear();
        comboBox_localization_type->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "AMCL", 0)
         << QApplication::translate("MainWindow", "Fake_localization", 0)
        );
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "Navigation", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
