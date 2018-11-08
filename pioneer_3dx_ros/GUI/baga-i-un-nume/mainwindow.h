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

    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    QProcess *myProcess;
};

#endif // MAINWINDOW_H
