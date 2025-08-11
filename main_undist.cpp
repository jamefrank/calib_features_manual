#include "include/cmdline.h"
#include "include/utils.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include <boost/filesystem.hpp>


int main(int argc, char *argv[])
{
    //
    spdlog::info("Welcome to use undist image tool!");

    //
    cmdline::parser parser;
    parser.add<std::string>("input", 'i', "Input directory containing images and config yaml", true, "");
    parser.add<std::string>("extension", 'e', "File extension of images", false, ".jpg");
    parser.add<std::string>("output", 'o', "Output dir", true, "");
    parser.add<std::string>("camera-model", 't', "Camera model", false, "pinhole", cmdline::oneof<std::string>("pinhole", "fisheye"));
    parser.add("verbose", '\0', "verbose when manual features");

    parser.parse_check(argc, argv);

    //
    std::string inputDir = parser.get<std::string>("input");
    std::string fileExtension = parser.get<std::string>("extension");
    std::string outputDir = parser.get<std::string>("output");
    bool verbose = parser.exist("verbose");

    //
    std::string camera_model = parser.get<std::string>("camera-model");
    std::string config_path = inputDir + "/config.yaml";
    YAML::Node config = YAML::LoadFile(config_path);
    cv::Mat K, D;
    undist::utils::parse_K(config, "K", K);
    undist::utils::parse_D(config, "D", D);
    undist::utils::log_cvmat(K, "K");
    undist::utils::log_cvmat(D, "D");

    //
    for (boost::filesystem::directory_iterator itr(inputDir); itr != boost::filesystem::directory_iterator(); ++itr) {
      if (!boost::filesystem::is_regular_file(itr->status())) {
          continue;
      }
      std::string filename = itr->path().filename().string();
      // check if file extension matches
      if (filename.compare(filename.length() - fileExtension.length(), fileExtension.length(), fileExtension) != 0) {
          continue;
      }
      //
      cv::Mat src = cv::imread(itr->path().string());
      cv::Mat undist_img;
      if("pinhole" == camera_model){
        undist_img = undist::utils::undist_image(src, K, D, undist::CameraType::PINHOLE);
      }
      else if("fisheye" == camera_model){
        undist_img = undist::utils::undist_image(src, K, D, undist::CameraType::FISHEYE);
      }
      std::string out_path = outputDir + "/" + filename;
      cv::imwrite(out_path, undist_img);

      spdlog::info("undist image {}", filename);
    }

}