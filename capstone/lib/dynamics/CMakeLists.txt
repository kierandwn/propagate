# CMakeList.txt : CMake component 'dynamics' for project 'propagate', include source and define
# component specific logic here.
#
cmake_minimum_required (VERSION 3.8)

include_directories(
	../config/include
    ../attitude/include
	../control/include
    ../telemetry/include
    ./include
)

add_library(model src/model.cc)
target_include_directories(model PRIVATE
	"./include"
	"../config/lib/yaml-cpp/include"
)
target_link_libraries(model
	config
	yaml-cpp
)

add_library(dynamics src/dynamics.cc)
target_include_directories(dynamics PRIVATE
	"./include"
	"../config/include"
	"../config/lib/yaml-cpp/include"
)
target_link_libraries(dynamics
    model
	config
	yaml-cpp
)