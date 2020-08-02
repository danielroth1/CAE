cmake_minimum_required(VERSION 3.0)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules")

project(CAE)

cmake_policy(SET CMP0054 OLD)

# Download all external libraries (cgal and eigen) and puts them in
# <CAE-source-dir>/extern/*. Checks if they are already there so this call only
# takes time the first time it's called. Currently only works with header only
# libraries.
execute_process(
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/linux/download_external_libs.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# Automatically downloads all assets that aren't there yet and puts them in
# <build-dir>/assets/*. This call only takes some time the first time it's
# called. The check if a file is already there is very cheap. It's executed
# each time qmake is run.
execute_process(COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/linux/download_assets.sh) 


## Global settings
set(CMAKE_CXX_STANDARD 14)
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON) 

# Visual studio solution directories.
set_property(GLOBAL PROPERTY USE_FOLDERS on)

# Enable simultaneous compilation of source files for MSVC.
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

# Enable C++11 support for GNU g++.
if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -fopenmp")
endif()

if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	add_definitions (-DCMAKE_CXX_COMPILER_ARG1=-std=c++1y)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")



# Put all executables and libraries into a common directory.
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}")
set(LIBRARY_OUTPUT_PATH    "${PROJECT_BINARY_DIR}")

add_subdirectory(src)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")

