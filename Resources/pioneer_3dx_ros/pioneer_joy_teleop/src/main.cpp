#include "pioneer_joy_teleop.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Pioneer_Joy_Teleop w;
    w.show();

    return a.exec();
}
