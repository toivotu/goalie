cmake_minimum_required (VERSION 2.8) 

#set (CMAKE_C_COMPILER gcc)
#set (CMAKE_CXX_COMPILER g++)

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -pedantic -s -O2 -march=native" )

project (GOALIE) 

include_directories (${GOALIE_SOURCE_DIR}/opencv/include)
link_directories (${GOALIE_SOURCE_DIR}/opencv/lib)
#link_directories (opencv/lib)

SET (OPENCV-VER 246d)

add_executable (goalie 
    goalie.cpp 
    balldetector.cpp
    hsvfilter.cpp
    rgbfilter.cpp)

add_custom_command(TARGET goalie POST_BUILD
    COMMAND cp ${GOALIE_SOURCE_DIR}/opencv/bin/opencv_ffmpeg246.dll ${EXECUTABLE_OUTPUT_PATH}/opencv_ffmpeg246.dll
)

target_link_libraries (goalie 
    opencv_core${OPENCV-VER}
    opencv_imgproc${OPENCV-VER}
    opencv_highgui${OPENCV-VER}
    opencv_objdetect${OPENCV-VER}
) 