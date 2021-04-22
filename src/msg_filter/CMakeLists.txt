cmake_minimum_required(VERSION 3.0.2)
project(msg_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  message_generation
  message_filters
  robotics_hw1
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    SpeedAndOdom.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
)




catkin_package(

CATKIN_DEPENDS
    std_msgs
    nav_msgs
    message_runtime
    message_filters
    robotics_hw1
)


include_directories(

  ${catkin_INCLUDE_DIRS}
)


add_executable(msg_filter src/msg_filter.cpp)
target_link_libraries(msg_filter ${catkin_LIBRARIES})
