set(TEST_NAME "${THEPROJECT_NAME}Test")

#include in the build all the content inside the directory
include_directories("../include")
include_directories("../../main/include")

#you might want to add the sources via the following command: set(SOURCES src/mainapp.cpp src/Student.cpp)
#but with GLOB is all much easier; include in the build all the content filtered by the pattern
file(GLOB SOURCES "*.cpp")


add_executable(${TEST_NAME} ${SOURCES})
link_directories(${CMAKE_BINARY_DIR})

if(${THEPROJECT_OUTPUT} STREQUAL "SO")
    message(STATUS "Building Tests against shared library")
    
    target_link_libraries(${TEST_NAME} ${PROJECT_NAME} ${THEPROJECT_TEST_ADDITIONAL_SHARED_LIBRARIES})
endif(${THEPROJECT_OUTPUT} STREQUAL "SO")

if(${THEPROJECT_OUTPUT} STREQUAL "AO")
    message(STATUS "Building Tests against static library")   
     
    target_link_libraries(${TEST_NAME} ${PROJECT_NAME} ${THEPROJECT_TEST_ADDITIONAL_SHARED_LIBRARIES})
endif(${THEPROJECT_OUTPUT} STREQUAL "AO")

set_target_properties(${TEST_NAME}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

######################## COPY RESOURCES ########################

#copy the contents of src/test/resources inside build/XXX
add_custom_command(
    TARGET ${TEST_NAME}
    POST_BUILD COMMAND 
    ${CMAKE_COMMAND} -E copy_directory ${CMAKE_BINARY_DIR}/../../src/test/resources $<TARGET_FILE_DIR:${THEPROJECT_NAME}>
)

####################### DONE MESSAGE ############################

add_custom_target(FinalMessage ALL
    ${CMAKE_COMMAND} -E cmake_echo_color --green --bold "========================== Compilation is successful! =========================="
    COMMENT "Final Message" 
)
add_dependencies(FinalMessage ${TEST_NAME} )

