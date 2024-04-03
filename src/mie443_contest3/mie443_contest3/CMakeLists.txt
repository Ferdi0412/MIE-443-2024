cmake_minimum_required(VERSION 2.8)
project(mie443_contest3)

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge 
    image_transport 
    sensor_msgs
    roscpp
)

catkin_package()

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# Add the publisher example
add_executable(publisher src/my_publisher_video.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Add the subscriber example
add_executable(contest3 src/contest3.cpp)
target_link_libraries(contest3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Add the response_test example
add_executable(response_test src/priv-response-test.cpp)
target_link_libraries(response_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Add the image server
add_executable(image_server src/image_handler/image_server.cpp)
target_link_libraries(image_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Add KinectFaceDetector
# add_executable(kinect_face_detector src/kinect_face_detector.cpp)
# target_link_libraries(kinect_face_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Specify C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
