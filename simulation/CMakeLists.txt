cmake_minimum_required(VERSION 3.12)
project(box-pushing VERSION 1.0.0 LANGUAGES CXX)
find_package(atta 0.4.0 REQUIRED EXACT)

# Components
atta_add_target(pusher_component "src/pusherComponent.cpp")

# Common
atta_add_target(pusher_common "src/pusherCommon.cpp")
target_link_libraries(pusher_common PRIVATE pusher_component)

# Scripts
atta_add_target(pusher_script "src/pusherScript.cpp")
target_link_libraries(pusher_script PRIVATE pusher_component pusher_common)
atta_add_target(pusher_paper_script "src/pusherPaperScript.cpp")
target_link_libraries(pusher_paper_script PRIVATE pusher_component pusher_common)
atta_add_target(pusher_teleop_script "src/pusherTeleopScript.cpp")
target_link_libraries(pusher_teleop_script PRIVATE pusher_component pusher_common)

# Project script
atta_add_target(project_script "src/projectScript.cpp")
target_link_libraries(project_script PRIVATE pusher_component)
