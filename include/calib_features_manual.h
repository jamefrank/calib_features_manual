#pragma once

#include <QMainWindow>
#include <QImage>
#include <opencv2/core.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <qevent.h>
#include <qimage.h>
#include "include/hi_imagebox.h"
#include "include/overlay_graphicsview.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class calib_features_manual;
}
QT_END_NAMESPACE

class calib_features_manual : public QMainWindow {
  Q_OBJECT

public:
  calib_features_manual(QWidget *parent = nullptr);
  ~calib_features_manual();
  void setPairs(std::vector<std::pair<std::string, std::string>>&& pairs);

public:
    void pointPickCallback(const pcl::visualization::PointPickingEvent& event, void* args);

protected:
  void initUi();
  void initOverlay();
  void showPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  void actImageClick(int x, int y);

  void resizeEvent(QResizeEvent* event);

private:
  Ui::calib_features_manual *ui;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr_;
  HI_ImageBox image_box_;
  std::vector<std::pair<std::string, std::string>> pairs_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi_ptr_;
  cv::Mat img_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr clicked_points_3d;

  OverlayGraphicsView *overlayView;
};
