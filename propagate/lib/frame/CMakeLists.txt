# CMakeList.txt : CMake component 'dynamics' for project 'propagate', include source and define
# component specific logic here.
#
cmake_minimum_required (VERSION 3.8)

include_directories("C:/Users/kdwn/projects/propagate/propagate/lib/frame/include")

ADD_LIBRARY(frame 
    frame.cc
)