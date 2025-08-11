#include "calib_features_manual.h"
#include "include/cmdline.h"
#include "include/utils.h"
#include <cassert>
#include <spdlog/spdlog.h>

#include <QApplication>
#include <qt5/QtWidgets/qapplication.h>
#include <string>


int main(int argc, char *argv[])
{
    //
    spdlog::info("Welcome to use calib features manual calib tool!");

    //
    cmdline::parser parser;
    parser.add<std::string>("img", 'i', "Input directory containing images", true, "");
    parser.add<std::string>("extension", 'e', "File extension of images", false, ".jpg");
    parser.add<std::string>("pcd", 'p', "Input directory containing pcds", true, "");
    parser.add<std::string>("output", 'o', "Output dir", true, "");
    parser.add<double>("time-thresh", 't', "time align thresh", false, 0.05);
    parser.add("time-align", '\0', "enable time align when manual features");
    parser.add("verbose", '\0', "verbose when manual features");

    parser.parse_check(argc, argv);

    //
    std::string img_dir = parser.get<std::string>("img");
    std::string pcd_dir = parser.get<std::string>("pcd");
    std::string img_ext = parser.get<std::string>("extension");
    std::string output_dir = parser.get<std::string>("output");
    double time_thresh = parser.get<double>("time-thresh");
    bool enable_time_align = parser.exist("time-align");
    bool verbose = parser.exist("verbose");

    //
    spdlog::info("finding time align ...");
    std::vector<std::pair<std::string, std::string>> pairs;
    bool bsuc = features_manual::utils::find_align_pcd_img(img_dir, pcd_dir, img_ext, enable_time_align, time_thresh, pairs);
    assert(bsuc);
    spdlog::info("Align Pairs: {}", pairs.size());

    //
    QApplication a(argc, argv);
    calib_features_manual w;
    w.setPairs(std::move(pairs));
    w.show();
    return a.exec();
}
