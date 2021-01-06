# CMakeList.txt : CMake project for propagate, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

add_subdirectory(lib/attitude)
add_subdirectory(lib/dynamics)
add_subdirectory(lib/telemetry)
add_subdirectory(lib/control)

# Add source to this project's executable.
add_executable (propagate "main.cpp" "lib/control/include/control.h")
target_link_libraries(propagate 
	dynamics 
	telemetry 
	control
)

# TODO: Add tests and install targets if needed.
