cmake_minimum_required(VERSION 2.8.3)
project(ski_main)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)



add_message_files(FILES dynamixel_info.msg)


generate_messages(DEPENDENCIES std_msgs)




catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ski_main
  CATKIN_DEPENDS roscpp std_msgs
 ## DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


 add_executable(${PROJECT_NAME}_node src/ski_main_node.cpp)
 add_library(${PROJECT_NAME} src/test_dynamixel_node.cpp)
 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(${PROJECT_NAME}_node ski_main_generate_messages_cpp)
 target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ski_main /usr/local/lib/libdxl_x64_cpp.so )
