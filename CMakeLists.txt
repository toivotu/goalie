cmake_minimum_required (VERSION 2.8) 

#set (CMAKE_C_COMPILER gcc)
#set (CMAKE_CXX_COMPILER g++)

project (OPENCV-TEST) 

include_directories (${OPENCV-TEST_SOURCE_DIR}/opencv/include)
link_directories (${OPENCV-TEST_SOURCE_DIR}/opencv/lib)
#link_directories (opencv/lib)

SET (OPENCV-VER 246d)

add_executable (opencv-test opencv-test.cpp balldetector.cpp)

target_link_libraries (opencv-test 
    opencv_core${OPENCV-VER}
    opencv_imgproc${OPENCV-VER}
    opencv_highgui${OPENCV-VER}
    opencv_objdetect${OPENCV-VER}
) 