cmake_minimum_required(VERSION 3.1.0)

add_compile_options(-std=c++14)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5UiTools 5.9 REQUIRED)
find_package(Qt5Widgets 5.9 REQUIRED)
find_package(Qt5Charts 5.9 REQUIRED)

add_library(chart_widget STATIC chart.cpp chart_resources.qrc)
target_link_libraries(chart_widget PUBLIC Qt5::Widgets Qt5::Charts Qt5::UiTools)
