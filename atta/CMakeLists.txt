cmake_minimum_required(VERSION 3.12)
project(box-pushing VERSION 1.0.0 LANGUAGES CXX)
find_package(atta 0.3.0.0 REQUIRED)

# Components
atta_add_target(pusher_component "src/pusherComponent.cpp")

# Scripts
atta_add_target(pusher_script "src/pusherScript.cpp")
target_link_libraries(pusher_script PRIVATE pusher_component)
atta_add_target(project_script "src/projectScript.cpp")
