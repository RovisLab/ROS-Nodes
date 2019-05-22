#include "pioneer_key_teleop.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Pioneer_Key_Teleop w;
    w.show();

    return a.exec();
}
