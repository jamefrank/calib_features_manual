#include "utils.h"
#include <cassert>
#include <limits>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <opencv2/calib3d/calib3d.hpp> 

bool features_manual::utils::find_align_pcd_img(const std::string& img_dir, const std::string& pcd_dir, const std::string& img_ext, bool b_enable_time_align, double time_thresh, std::vector<std::pair<std::string, std::string>>& pairs) {
    //
    std::vector<std::string> pcd_files = get_file_list(pcd_dir, ".pcd");
    std::vector<std::string> img_files = get_file_list(pcd_dir, img_ext);
    assert(pcd_files.size()>0);
    assert(img_files.size()>0);

    //
    if(b_enable_time_align){
        int pcd_idx=0, img_idx=0;
        double pcd_ts = get_timestamp(pcd_files[pcd_idx]);
        double img_ts = get_timestamp(img_files[img_idx]);
        double ts_diff = std::abs(pcd_ts-img_ts);
        while(pcd_idx<pcd_files.size() && img_idx<img_files.size()){
            if(pcd_ts < img_ts){
                pcd_idx++;
                if(pcd_idx >= pcd_files.size()) continue;
                pcd_ts = get_timestamp(pcd_files[pcd_idx]);
                double tmp_ts_diff = std::abs(pcd_ts-img_ts);
                if(tmp_ts_diff >= ts_diff){
                    if (ts_diff<time_thresh){
                        pairs.push_back(std::make_pair(pcd_dir+"/"+pcd_files[pcd_idx], img_dir+"/"+img_files[img_idx]));
                        std::string pcd_name = std::filesystem::path(pcd_files[pcd_idx]).filename().string();
                        std::string img_name = std::filesystem::path(img_files[img_idx]).filename().string();
                        spdlog::info("[align] pcd:{}, img:{}", pcd_name, img_name);
                    }
                    else
                        ts_diff = std::numeric_limits<double>::max();
                }
                else
                    ts_diff = tmp_ts_diff;
            }
            else if(pcd_ts > img_ts){
                img_idx++;
                if(img_idx >= img_files.size()) continue;
                img_ts = get_timestamp(img_files[img_idx]);
                double tmp_ts_diff = std::abs(pcd_ts-img_ts);
                if(tmp_ts_diff >= ts_diff){
                    if (ts_diff<time_thresh){
                        pairs.push_back(std::make_pair(pcd_dir+"/"+pcd_files[pcd_idx], img_dir+"/"+img_files[img_idx]));
                        std::string pcd_name = std::filesystem::path(pcd_files[pcd_idx]).filename().string();
                        std::string img_name = std::filesystem::path(img_files[img_idx]).filename().string();
                        spdlog::info("[align] pcd:{}, img:{}", pcd_name, img_name);
                    }
                    else
                        ts_diff = std::numeric_limits<double>::max();
                }
                else
                    ts_diff = tmp_ts_diff;
            } 
            else{
                pairs.push_back(std::make_pair(pcd_dir+"/"+pcd_files[pcd_idx], img_dir+"/"+img_files[img_idx]));
                std::string pcd_name = std::filesystem::path(pcd_files[pcd_idx]).filename().string();
                std::string img_name = std::filesystem::path(img_files[img_idx]).filename().string();
                spdlog::info("[align] pcd:{}, img:{}", pcd_name, img_name);
                pcd_idx++;
                if(pcd_idx >= pcd_files.size()) continue;
                pcd_ts = get_timestamp(pcd_files[pcd_idx]);
                img_idx++;
                if(img_idx >= img_files.size()) continue;
                img_ts = get_timestamp(img_files[img_idx]);
                ts_diff = std::abs(pcd_ts-img_ts);
            }
        }
        if(ts_diff < time_thresh){
            pairs.push_back(std::make_pair(pcd_dir+"/"+pcd_files[pcd_idx], img_dir+"/"+img_files[img_idx]));
            std::string pcd_name = std::filesystem::path(pcd_files[pcd_idx]).filename().string();
            std::string img_name = std::filesystem::path(img_files[img_idx]).filename().string();
            spdlog::info("[align] pcd:{}, img:{}", pcd_name, img_name);
        }

        return pairs.size()>0;
    }
    else{
        for(int i=0; i<pcd_files.size(); i++){
            pairs.push_back(std::make_pair(pcd_dir+"/"+pcd_files[i], img_dir+"/"+img_files[i]));
            std::string pcd_name = std::filesystem::path(pcd_files[i]).filename().string();
            std::string img_name = std::filesystem::path(img_files[i]).filename().string();
            spdlog::info("[align] pcd:{}, img:{}", pcd_name, img_name);
        }
        return pcd_files.size()==img_files.size();
    }

    return true;
}

std::vector<std::string> features_manual::utils::get_file_list(std::string folderPath, std::string targetExtension) {
    std::vector<std::string> fileList;
    if (std::filesystem::exists(folderPath) && std::filesystem::is_directory(folderPath)) {
        for (const auto& entry : std::filesystem::directory_iterator(folderPath)) {
            if (entry.is_regular_file()) {
                std::string fileExtension = entry.path().extension().string();
                if (fileExtension == targetExtension) {
                    fileList.push_back(entry.path().filename().string());
                }
            }
        }

        std::sort(fileList.begin(), fileList.end());
    }
    return fileList;
}

double features_manual::utils::get_timestamp(const std::string& abs_file_path) {
    auto filename = std::filesystem::path(abs_file_path).stem().string();
    double timestamp = std::stod(filename);

    return timestamp;
}

bool features_manual::utils::load_pcd(const std::string& pcd_path, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_xyzi_ptr_) {
    pcl::PCLPointCloud2 cloud2;
    if (pcl::io::loadPCDFile(pcd_path, cloud2) == -1) {
        PCL_ERROR("Failed to load PCD file into PCLPointCloud2!\n");
        return false;
    }
    else
        spdlog::info("load pcd 2 PCLPointCloud2 success");
    pcl::fromPCLPointCloud2(cloud2, *cloud_xyzi_ptr_);
    spdlog::info("cloud point size: {}", cloud_xyzi_ptr_->points.size());
    auto minmax = std::minmax_element(
        cloud_xyzi_ptr_->begin(),
        cloud_xyzi_ptr_->end(),
        [](const pcl::PointXYZI& a, const pcl::PointXYZI& b) {
            return a.intensity < b.intensity;
        }
    );
    float min_intensity = minmax.first->intensity;
    float max_intensity = minmax.second->intensity;
    spdlog::info("Intensity range: min = {:.2f}, max = {:.2f}", min_intensity, max_intensity);
    if(max_intensity == 0){
        spdlog::info("calc intensity by distance");
        for (auto& point : *cloud_xyzi_ptr_) {
            float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            point.intensity = distance;
        }
    }

    return true;
}

bool undist::utils::parse_K(const YAML::Node& config, std::string name, cv::Mat& K) {
    if(config[name]){
        if(config[name].IsSequence()){
            int num = config[name].size();
            if(9==num || 3==num){
                std::vector<float> tmp;   
                if(9 == num){
                    for(int i=0; i<num; i++){
                        float value = config[name][i].as<float>();
                        tmp.push_back(value);
                    }
                }
                if(3 == num){
                    for(int i=0; i<num; i++){
                        for (int j = 0; j < config[name][i].size(); ++j) {
                            float value = config[name][i][j].as<float>();
                            tmp.push_back(value);
                        }
                    }
                }
                K = cv::Mat(3, 3, CV_32F, tmp.data()).clone();
                return true;
            }
            else
                spdlog::error("3x3 or 9 supported");
        }
        else
            spdlog::error("[{0}] not list", name);
    }
    else{
        spdlog::error("{0} not exists", name);
    }
    
    return false;
}

void undist::utils::parse_D(const YAML::Node& config, std::string name, cv::Mat& D_) {
    std::vector<float> D;   //TODO
    for (std::size_t i = 0; i < config["D"].size(); ++i) {
        float value = config["D"][i].as<float>();
        D.push_back(value);
    }
    D_ = cv::Mat(1, D.size(), CV_32F, D.data()).clone();
}

void undist::utils::log_cvmat(const cv::Mat& mat, const std::string& name) {
    std::ostringstream oss;
    oss << name << " = " << std::endl << mat << std::endl;
    spdlog::info("{}", oss.str());
}

cv::Mat undist::utils::undist_image(const cv::Mat& src, const cv::Mat& K, const cv::Mat& D, CameraType type) {
    cv::Mat dst;
    if(type == CameraType::PINHOLE){
        cv::undistort(src, dst, K, D);
    }
    else if(type ==CameraType::FISHEYE){
        cv::Mat map1;
        cv::Mat map2;
        cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat(), K, src.size(), CV_16SC2, map1, map2);
        cv::remap(src, dst, map1, map2, cv::INTER_AREA, cv::BORDER_CONSTANT);
    }
    else{
        spdlog::error("unknown camera type");
        exit(-1);
    }

    return dst;
}
