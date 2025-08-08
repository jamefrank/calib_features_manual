#include "calib_features_manual.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    calib_features_manual w;
    w.show();
    return a.exec();
}
