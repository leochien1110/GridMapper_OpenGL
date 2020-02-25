cmake_minimum_required (VERSION 3.2)

# The project name
project (mapper_px4)

if(WIN32)
	# set cmake module for OpenGL
	set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
	# set OpenCV directory for find_package
	#set("OpenCV_DIR" "C:/opencv/build/install")
elseif(UNIX)
	set(CMAKE_CXX_FLAGS "-lncurses")
endif()

add_subdirectory(thirdparty)
add_subdirectory(res)
add_subdirectory(src)
