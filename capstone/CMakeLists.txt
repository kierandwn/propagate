# CMakeList.txt : CMake project for propagate, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

add_subdirectory(lib/attitude)
add_subdirectory(lib/dynamics)
add_subdirectory(lib/telemetry)
add_subdirectory(lib/control)
add_subdirectory(lib/config)

include_directories(
	lib/config/include
	lib/attitude/include
	lib/control/include
	lib/dynamics/include
	lib/telemetry/include
)

# Add source to this project's executable.
add_executable (propagate "main.cpp")
target_link_libraries(propagate 
	dynamics 
	model
	telemetry 
	control
	config
)

# TODO: Add tests and install targets if needed.