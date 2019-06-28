#include "pioneer_gui.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Pioneer_Gui w;
    //w.show();
    w.showMaximized();

    return a.exec();
}
