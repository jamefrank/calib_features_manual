#include "calib_features_manual.h"
#include "./ui_calib_features_manual.h"
#include "include/utils.h"
#include "include/myvtk_interactor_style.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <QPixmap>
#include <qboxlayout.h>
#include <qchar.h>
#include <qglobal.h>
#include <qimage.h>
#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>
#include <QDebug>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QtWidgets/QSpacerItem>

#include <vtkRenderWindow.h>

calib_features_manual::calib_features_manual(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::calib_features_manual)
{
    ui->setupUi(this);
    //
    initUi();
}

calib_features_manual::~calib_features_manual()
{
    delete ui;
}

void calib_features_manual::setPairs(std::vector<std::pair<std::string, std::string>>&& pairs) {
    pairs_ = std::move(pairs);

    //
    features_manual::utils::load_pcd(pairs_[0].first, cloud_xyzi_ptr_);
    
    //
    showPointCloud(cloud_xyzi_ptr_);

    //
    image_box_.SetImage(QString::fromStdString(pairs_[0].second));
}

void calib_features_manual::pointPickCallback(const pcl::visualization::PointPickingEvent& event, void* args) {
    if (event.getPointIndex() == -1) return;


    pcl::PointXYZI current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    clicked_points_3d->points.push_back(current_point);

    // 显示选中的点
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> red(clicked_points_3d, 255, 0, 0);
    viewer_ptr_->removePointCloud("clicked_points");
    viewer_ptr_->addPointCloud(clicked_points_3d, red, "clicked_points");
    viewer_ptr_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");

    ui->qvtkWidget->update();
}

void calib_features_manual::initUi(){
    //
    // image_box_.Init(ui->verticalLayout);
    // connect(&image_box_, &HI_ImageBox::ImageClick, this, &calib_features_manual::actImageClick);
    //
    viewer_ptr_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    viewer_ptr_->addCoordinateSystem();
    ui->qvtkWidget->SetRenderWindow(viewer_ptr_->getRenderWindow());
    viewer_ptr_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();
    auto renderer = ui->qvtkWidget->GetRenderWindow()->GetRenderers()->GetFirstRenderer();
    renderer->SetViewport(0.5, 0.0, 1.0, 1.0); 

    // vtkRenderWindowInteractor* interactor =  ui->qvtkWidget->GetInteractor();
    // interactor->SetInteractorStyle(MyVtkInteractorStyle::New());

    clicked_points_3d.reset(new pcl::PointCloud<pcl::PointXYZI>());
    viewer_ptr_->registerPointPickingCallback(&calib_features_manual::pointPickCallback, *this);

    //
    cloud_xyzi_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    //
    QHBoxLayout *horizontalLayout = new QHBoxLayout(ui->qvtkWidget);
    QVBoxLayout *verticalLayout = new QVBoxLayout();
    horizontalLayout->addLayout(verticalLayout);
    QSpacerItem *horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
    horizontalLayout->addSpacerItem(horizontalSpacer);

    horizontalLayout->setStretch(0, 1);
    horizontalLayout->setStretch(1, 1);

    image_box_.Init(verticalLayout);
    connect(&image_box_, &HI_ImageBox::ImageClick, this, &calib_features_manual::actImageClick);
}





void calib_features_manual::showPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    //
    viewer_ptr_->removeAllPointClouds();
    viewer_ptr_->removeAllShapes();
    //
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_color(cloud, 255, 0, 0);
    // viewer_ptr_->addPointCloud(cloud, cloud_color, "lidar");

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_color(cloud, "intensity");
    viewer_ptr_->addPointCloud(cloud, intensity_color, "lidar");

    //
    ui->qvtkWidget->update();
}

void calib_features_manual::actImageClick(int x, int y) {
  spdlog::info("image click point: {},{}", x, y);
}




