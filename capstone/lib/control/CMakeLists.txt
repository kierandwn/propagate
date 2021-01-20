cmake_minimum_required (VERSION 3.8)

include_directories(
    ../attitude/include
    ../telemetry/include
    ./include
)

ADD_LIBRARY(frame
    src/frame.cc
)

ADD_LIBRARY(control 
    src/control.cc
)

target_link_libraries(control
	frame
)

add_subdirectory("test")
