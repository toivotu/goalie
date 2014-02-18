cmake_minimum_required (VERSION 2.8) 

project (GOALIE)

if (WIN32) 
    INCLUDE ("CMakeLists_win.txt")
else()
    INCLUDE ("CMakeLists_linux.txt")
endif (WIN32)