cmake_minimum_required(VERSION 3.10)

# Set the project name
project(fxpbd)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the include directory
include_directories(${PROJECT_SOURCE_DIR}/inc)

link_libraries(sfml-graphics sfml-window sfml-system ${OpenCV_LIBS})

# Collect all .cpp files in the ./test/ directory
file(GLOB TEST_SOURCES ${PROJECT_SOURCE_DIR}/test/*.cpp)

# Loop through each source file and create an executable
foreach(TEST_SOURCE ${TEST_SOURCES})
    # Get the filename without the directory
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    # Create an executable with the same name as the source file
    add_executable(${TEST_NAME} ${TEST_SOURCE})
endforeach()
