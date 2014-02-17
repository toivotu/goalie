cmake_minimum_required (VERSION 2.8) 
project (BALLTRACE) 

#set (CMAKE_C_COMPILER gcc)
#set (CMAKE_CXX_COMPILER g++)
#set (CMAKE_CXX_FLAGS (${CMAKE_CXX_FLAGS} -Wall))


include_directories (${BALLTRACE_SOURCE_DIR})
link_directories (/usr/lib/uv4l/uv4lext/armv6l/ ${BALLTRACE_SOURCE_DIR}/raspicamcv /opt/vc/lib/ /usr/local/lib)
#link_directories (opencv/lib)

SET (OPENCV-VER "")

add_executable (follow 
    ${BALLTRACE_SOURCE_DIR}/follow.cpp
    ${BALLTRACE_SOURCE_DIR}/hsvfilter.cpp
    ${BALLTRACE_SOURCE_DIR}/balldetector.cpp
    ${BALLTRACE_SOURCE_DIR}/serial/serial.cc
    ${BALLTRACE_SOURCE_DIR}/serial/unix.cc
)

add_executable (webui
    ${BALLTRACE_SOURCE_DIR}/webui.cpp
)

target_link_libraries (webui
    websock
)

target_link_libraries (follow
    opencv_core${OPENCV-VER}
    opencv_imgproc${OPENCV-VER}
    opencv_highgui${OPENCV-VER}
    #uv4lext
    ncurses
    raspicamcv
    
    mmal_core
    mmal
    mmal_util
    vcos
    bcm_host
) 