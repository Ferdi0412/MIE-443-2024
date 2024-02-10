cmake_minimum_required(VERSION 2.8)
project(mie443_contest1)

add_compile_options(-std=c++11)

#set(OpenCV_DIR "/usr/share/OpenCV")

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	genmsg
	image_transport
	sensor_msgs
	kobuki_msgs)

generate_messages(DEPENDENCIES sensor_msgs kobuki_msgs)

catkin_package()

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add the publisher example
add_executable(contest1 src/contest1.cpp)
target_link_libraries(contest1 ${catkin_LIBRARIES} ${OpenCV_LIB})

# Add personal testing stuff.....
add_executable(parin_test src/parin_test.cpp)
target_link_libraries(parin_test ${catkin_LIBRARIES} ${OpenCV_LIB})

add_executable(ajeya_test src/ajeya_test.cpp)
target_link_libraries(ajeya_test ${catkin_LIBRARIES} ${OpenCV_LIB})

#add_executable(emma_test src/emma_test.cpp)
#target_link_libraries(emma_test ${catkin_LIBRARIES} ${OpenCV_LIB})

add_executable(ferdi_test src/ferdi_test.cpp)
target_link_libraries(ferdi_test ${catkin_LIBRARIES} ${OpenCV_LIB})
