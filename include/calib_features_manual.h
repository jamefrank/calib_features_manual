#pragma once

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui {
class calib_features_manual;
}
QT_END_NAMESPACE

class calib_features_manual : public QMainWindow
{
    Q_OBJECT

public:
    calib_features_manual(QWidget *parent = nullptr);
    ~calib_features_manual();

private:
    Ui::calib_features_manual *ui;
};
