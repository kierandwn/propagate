# CMakeList.txt : CMake component 'dynamics' for project 'propagate', include source and define
# component specific logic here.
#
cmake_minimum_required (VERSION 3.8)

set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "disable yaml tools")
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "disable yaml contrib")

add_subdirectory(lib/yaml-cpp)

add_library(config src/config.cc)
target_include_directories(config PRIVATE
    "./include"
	"../lib/yaml-cpp/include"
)
target_link_libraries(config
	yaml-cpp
)