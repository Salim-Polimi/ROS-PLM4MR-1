cmake_minimum_required(VERSION 3.0.2)
project(msg_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  message_filters
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    MotorSpeed.msg
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
)




catkin_package(

CATKIN_DEPENDS
    std_msgs
    message_runtime
    message_filters
)


include_directories(

  ${catkin_INCLUDE_DIRS}
)

add_executable(msg_filter src/msg_filter_sub.cpp)
target_link_libraries(msg_filter ${catkin_LIBRARIES})







