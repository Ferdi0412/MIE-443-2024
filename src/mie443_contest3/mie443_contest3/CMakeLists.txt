cmake_minimum_required(VERSION 2.8)
project(mie443_contest3)

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
		cv_bridge
		image_transport
		sensor_msgs
        roscpp)

catkin_package()

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add the publisher example
add_executable(publisher src/my_publisher_video.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the subscriber example
add_executable(contest3 src/contest3.cpp
			src/imageTransporter.cpp)
target_link_libraries(contest3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the subscriber example
add_executable(response_test src/priv-response-test.cpp
			src/imageTransporter.cpp)
target_link_libraries(response_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Add the image server
add_executable(image_server src/image_handler/image_server.cpp
			src/imageTransporter.cpp)
target_link_libraries(image_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Add the image server
add_executable(face_server src/face_detect/face_server.cpp
			src/imageTransporter.cpp
			src/kinect_face_detector.cpp)
target_link_libraries(face_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
