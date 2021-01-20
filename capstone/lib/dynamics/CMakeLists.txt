# CMakeList.txt : CMake component 'dynamics' for project 'propagate', include source and define
# component specific logic here.
#
cmake_minimum_required (VERSION 3.8)

#include_directories("include")

ADD_LIBRARY(dynamics dynamics.cc)
target_include_directories(dynamics PRIVATE
	"./include"
	"../config/lib/yaml-cpp/include"
)
target_link_libraries(dynamics
	config
)