cmake_minimum_required(VERSION 2.8.3)
project(pca9685_board)
add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
)

add_message_files(FILES Servo.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

set(SOURCES
    src/main.cpp
    src/PCA9685Node.cpp
    src/PCA9685Controller.cpp
)

add_executable(${PROJECT_NAME}_node ${SOURCES})
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    i2c
)
add_dependencies(${PROJECT_NAME}_node pca9685_board_generate_messages_cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_node PREFIX "")
