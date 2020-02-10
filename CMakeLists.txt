##---------------------------------------------------------------------------------------------------------------------
##  OPTITRACK IMU COMPARATIVE
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2020 - Marco Montes Grova (a.k.a. mgrova) marrcogrova@gmail.com 
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
##  and associated documentation files (the "Software"), to deal in the Software without restriction, 
##  including without limitation the rights to use, copy, modify, merge, publish, distribute, 
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial 
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES 
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.7)
project(optitrack_IMU_comparative)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED roscpp geometry_msgs sensor_msgs )
if(${catkin_FOUND})
	catkin_package(
		CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs
		# INCLUDE_DIRS include
	)
endif(${catkin_FOUND})

find_package(serial REQUIRED)

find_package(Eigen3 REQUIRED)

set(SOURCE_FILES src/main.cpp )
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include) 

if(${catkin_FOUND})
	target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${catkin_LIBRARIES} )
	add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif(${catkin_FOUND})

if(${Eigen3_FOUND})
	target_include_directories(${PROJECT_NAME} PUBLIC "/usr/include/eigen3")
endif()

if(${serial_FOUND})
	target_include_directories(${PROJECT_NAME} PUBLIC ${serial_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${serial_LIBRARIES} )
endif(${serial_FOUND})
