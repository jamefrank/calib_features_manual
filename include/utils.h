#include <opencv2/core/mat.hpp>
#include <string>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>

namespace features_manual{
    namespace utils{
        bool find_align_pcd_img(const std::string& img_dir, const std::string& pcd_dir, const std::string& img_ext, bool b_enable_time_align, double time_thresh, std::vector<std::pair<std::string, std::string>>& pairs);
        std::vector<std::string> get_file_list(std::string folderPath, std::string targetExtension);
        double get_timestamp(const std::string& abs_file_path);

        bool load_pcd(const std::string& pcd_path, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_xyzi_ptr_);
    }
}




namespace undist {
    enum class CameraType {
        PINHOLE,
        FISHEYE
    };
    namespace utils{
        bool parse_K(const YAML::Node& config, std::string name, cv::Mat& K);
        void parse_D(const YAML::Node& config, std::string name, cv::Mat& D);
        void log_cvmat(const cv::Mat& mat, const std::string& name = "Mat");

        cv::Mat undist_image(const cv::Mat& src, const cv::Mat& K, const cv::Mat& D, CameraType type);
    }
}