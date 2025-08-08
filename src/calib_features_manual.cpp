#include "calib_features_manual.h"
#include "./ui_calib_features_manual.h"

calib_features_manual::calib_features_manual(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::calib_features_manual)
{
    ui->setupUi(this);
}

calib_features_manual::~calib_features_manual()
{
    delete ui;
}
