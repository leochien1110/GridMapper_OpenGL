project(mapper_px4)

# Preset
if (POLICY CMP0072)
  set(OpenGL_GL_PREFERENCE LEGACY)
endif()

# Find external package
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)
find_package (Threads)
#find_package(OpenMP)
find_package(X11 REQUIRED)

# copy resource folder to build
file(COPY ${CMAKE_SOURCE_DIR}/res DESTINATION ${CMAKE_BINARY_DIR})

# Set Path
set(OPENCV_LIBS_PATH ../../opencv/build/lib)
set(REALSENSE_LIBS_PATH ../../librealsense/build)

# copy resource folder to build
file(COPY ${CMAKE_SOURCE_DIR}/res DESTINATION ${CMAKE_BINARY_DIR})

# Include directory
include_directories(../thirdparty/include)
include_directories(../../librealsense/include)
# add RealSense and OpenCV include
include_directories(${OpenCV_INCLUDE_DIRS})

# setup OpenMP
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


# Setup RealSense
#set(REALSENSE_LIBS ${REALSENSE_LIBS} -lrealsense2)
find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBREALSENSE_DIR} REQUIRED)

if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()

# Link to library
link_directories(../thirdparty/lib)

# The main program
add_executable(onboard main.cpp
                       onboard.h
                       onboard.cpp
                       rs_camera.h
                       rs_camera.cpp
                       mapper.h
                       mapper.cpp
                       )

target_link_libraries(onboard PRIVATE thirdparty
                      pthread
                      ${CMAKE_THREAD_LIBS_INIT}
                      ${REALSENSE2_FOUND}
                      ${OpenCV_LIBS} 
                      ${CMAKE_DL_LIBS})

#[[
add_executable(onboard onboard.cpp
                       connection.hpp
                       connection.cpp
                       stb_image.h 
                       shader_m.h
                       glad.c)
                       
target_link_libraries(onboard PRIVATE thirdparty 
                       ${CMAKE_THREAD_LIBS_INIT} 
                       ${OpenCV_LIBS} 
                       ${REALSENSE2_FOUND} 
                       ${OPENGL_gl_LIBRARY} 
                       ${OPENGL_glu_LIBRARY} 
                       ${CMAKE_DL_LIBS} 
                       glfw3 
                       GL 
                       X11 
                       pthread Xrandr Xi dl ${CURSES_LIBRARIES})

add_executable(offboard offboard.cpp 
                        stb_image.h 
                        shader_m.h 
                        glad.c 
                        imgui/imgui.cpp 
                        imgui/imgui_draw.cpp 
                        imgui/imgui_demo.cpp
                        imgui/imgui_widgets.cpp 
                        imgui/imgui_impl_opengl3.cpp 
                        imgui/imgui_impl_glfw.cpp)

target_link_libraries(offboard PRIVATE thirdparty 
                      ${CMAKE_THREAD_LIBS_INIT} 
                      ${OpenCV_LIBS} 
                      ${OPENGL_gl_LIBRARY} 
                      ${OPENGL_glu_LIBRARY} 
                      ${CMAKE_DL_LIBS} 
                      glfw3 
                      GL 
                      X11 
                      pthread 
                      Xrandr 
                      Xi 
                      dl)
]]