cmake_minimum_required(VERSION 3.14)
project(calib_features_manual VERSION 0.1 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE Debug)  # just for debug

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(VTK REQUIRED)

set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)
include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)

file(GLOB QT_SOURCES    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
file(GLOB QT_UIS        RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB QT_HS         RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/*.h)

set(PROJECT_SOURCES
    main.cpp
    ${QT_SOURCES}
    ${QT_UIS}
    ${QT_RESOURCES}
    ${QT_HS}
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(calib_features_manual
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
    add_executable(calib_features_manual
        ${PROJECT_SOURCES}
    )
endif()

target_link_libraries(calib_features_manual PRIVATE 
    Qt${QT_VERSION_MAJOR}::Widgets
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
    spdlog::spdlog_header_only
    ${Boost_LIBRARIES}
    ${VTK_LIBRARIES}
)

if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.calib_features_manual)
endif()
set_target_properties(calib_features_manual PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(calib_features_manual)
endif()


# undist tool
add_executable(undistTool 
    main_undist.cpp
    src/utils.cpp
)
target_link_libraries(undistTool
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
    spdlog::spdlog_header_only
)