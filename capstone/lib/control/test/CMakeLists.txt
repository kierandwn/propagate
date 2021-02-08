set(TEST_EXEC capstone_test)

file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES false *.h *.cc)
set(SOURCES ${TEST_SOURCES})

add_executable(${TEST_EXEC} ${TEST_SOURCES})
target_link_libraries(${TEST_EXEC}
	frame
)

add_test(NAME ${TEST_EXEC} COMMAND ${TEST_EXEC})