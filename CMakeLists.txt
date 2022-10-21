cmake_minimum_required(VERSION 3.5)

project(wheebbot-packages VERSION 1.0.0)

add_subdirectory(wheebbot_utils)
add_subdirectory(wheebbot_state_estimation)
#add_subdirectory(wheebbot_results)
#add_subdirectory(wheebbot_os)
add_subdirectory(wheebbot_odrive)
add_subdirectory(wheebbot_gazebo)
#add_subdirectory(wheebbot_docs)
add_subdirectory(wheebbot_description)
add_subdirectory(wheebbot_cntrl)
#add_subdirectory(wheebbot_cads)
#add_subdirectory(wheebbot_arduino)

