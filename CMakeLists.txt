cmake_minimum_required(VERSION 3.0)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules")

project(CAE)

if (UNIX)
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
    execute_process(
		COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/linux/download_assets.sh
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

else (UNIX)
    # Same scripts as above but in python.
    execute_process(
        COMMAND python -B ${CMAKE_CURRENT_SOURCE_DIR}/scripts/python/download_external_libs.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
	# Assets are downloaded in the installation
    # execute_process(
	#	COMMAND python -B ${CMAKE_CURRENT_SOURCE_DIR}/scripts/python/download_assets.py
	#	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}) 
endif (UNIX)

## Global settings
set(CMAKE_CXX_STANDARD 14)
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
# For VS builds, runs INSTALL after the build of the Solution.
set(CMAKE_VS_INCLUDE_INSTALL_TO_DEFAULT_BUILD 1)

add_compile_definitions(_DISABLE_EXTENDED_ALIGNED_STORAGE)

# Visual studio solution directories.
set_property(GLOBAL PROPERTY USE_FOLDERS on)

# Enable simultaneous compilation of source files for MSVC.
if (MSVC)
    add_definitions(/MP)
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
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}")

# Global variables that can be used in other CMake files.
set(CMAKE_ROOT_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_ROOT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_EXTERN_DIR "${CMAKE_ROOT_SOURCE_DIR}/extern")

add_subdirectory(src)

# Sets CAE as startup project in Visual Studio
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT CAE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")

