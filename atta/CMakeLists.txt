cmake_minimum_required(VERSION 3.12)
project(box-pushing VERSION 1.0.0 LANGUAGES CXX)
find_package(atta 0.5.0.0 REQUIRED)

atta_add_target(pusher_script "src/pusherScript.cpp")
atta_add_target(project_script "src/projectScript.cpp")
