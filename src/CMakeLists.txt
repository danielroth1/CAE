
add_definitions( -DBOOST_PARAMETER_MAX_ARITY=12 )

# find required packages
find_package(OpenGL REQUIRED)

# Glut
if (UNIX)
	find_package(GLUT REQUIRED)
else (UNIX)
	set(GLUT_INCLUDE_DIR ${CMAKE_EXTERN_DIR}/freeglut/include)
	set(GLUT_LIBRARY ${CMAKE_EXTERN_DIR}/freeglut/lib/x64/freeglut.lib)
endif (UNIX)

# Eigen
set(EIGEN3_INCLUDE_DIR ${CMAKE_EXTERN_DIR}/eigen)
if (NOT EXISTS ${EIGEN3_INCLUDE_DIR})
    message("Error: Missing Eigen directory: ${EIGEN3_INCLUDE_DIR}. Try executing manually: bash scripts/linux/download_external_libs.sh")
endif (NOT EXISTS ${EIGEN3_INCLUDE_DIR})
# To use Eigen from the package management
#find_package(Eigen3 REQUIRED)

# Boost and its components
if (UNIX)
	find_package(Boost COMPONENTS system thread filesystem REQUIRED)
else (UNIX)
	option(USE_SYSTEM_BOOST "Search for a boost installation in the system and use that instead of downloading a reduced boost version." OFF)
	if (USE_CUSTOM_BOOST)
		# Custom Boost
		find_package(Boost COMPONENTS system thread filesystem REQUIRED)
	else (USE_CUSTOM_BOOST)
		# Download Boost if not already done.
		execute_process(
			COMMAND python -B ${CMAKE_ROOT_SOURCE_DIR}/scripts/python/download_boost.py
			WORKING_DIRECTORY ${CMAKE_ROOT_SOURCE_DIR})
		
		set(BOOST_ROOT ${CMAKE_ROOT_SOURCE_DIR}/extern/boost)
		set(BOOST_LIBRARYDIR ${CMAKE_ROOT_SOURCE_DIR}/extern/boost/lib)
		set(BOOST_INCLUDEDIR ${CMAKE_ROOT_SOURCE_DIR}/extern/boost/include)
		
		find_package(Boost REQUIRED COMPONENTS system thread filesystem)
		# find_package(Boost NO_DEFAULT_PATH REQUIRED COMPONENTS system thread filesystem PATHS ${CMAKE_ROOT_SOURCE_DIR}/extern/boost/lib/cmake/Boost-1.73.0)
	endif (USE_CUSTOM_BOOST)
endif (UNIX)

if (NOT Boost_FOUND)
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()  
endif()

# Gmp and Mpfr
# Is required by CGAL. For Linux it needs to be installed in the package management.
# For Windows the binaries are provided and downloaded.
if (WIN32)
	#find_package(gmp REQUIRED PATHS ${CMAKE_EXTERN_DIR}/gmp)
	set(GMP_LIBRARIES ${CMAKE_EXTERN_DIR}/gmp/lib/libgmp-10.lib)
	set(GMP_INCLUDE_DIR ${CMAKE_EXTERN_DIR}/gmp/include)

	set(MPFR_LIBRARIES ${CMAKE_EXTERN_DIR}/gmp/lib/libmpfr-4.lib)
	set(MPFR_INCLUDE_DIR ${CMAKE_EXTERN_DIR}/gmp/include)
endif (WIN32)

# CGAL
# CGAL and its components
# For CGAL custom cmake files are provided in the downloaded files. CMake only finds them if the search path is set manually.
find_package(CGAL COMPONENTS NO_DEFAULT_PATH PATHS ${CMAKE_EXTERN_DIR}/cgal)
if (NOT CGAL_FOUND)
  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  
endif()
include(${CGAL_USE_FILE}) # include helper file

# GLEW
if (UNIX)
	find_package(GLEW REQUIRED)
else (UNIX)
	set(GLEW_INCLUDE_DIR ${CMAKE_EXTERN_DIR}/glew/include)
	set(GLEW_LIBRARIES ${CMAKE_EXTERN_DIR}/glew/lib/Release/x64/glew32.lib)
	set(GLEW_SHARED_LIBRARY_RELEASE ${GLEW_LIBRARIES})
endif (UNIX)

# Qt
if (UNIX)
	find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL CONFIG)
else (UNIX)
	if (Qt5_DIR)
		# Custom Qt5
		find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL CONFIG PATHS ${Qt5_DIR})
	else (Qt5_DIR)
		# Download Qt binaries if not already done.
		execute_process(
			COMMAND python -B ${CMAKE_ROOT_SOURCE_DIR}/scripts/python/download_qt.py
			WORKING_DIRECTORY ${CMAKE_ROOT_SOURCE_DIR})
		set (Qt5_DIR ${CMAKE_ROOT_SOURCE_DIR}/extern/qt/lib/cmake/Qt5)
		find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL CONFIG PATHS ${Qt5_DIR})
	endif (Qt5_DIR)
endif (UNIX)

set(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_RELEASE} -fprofile-arcs -ftest-coverage")

# set up a mapping so that the Release configuration for the Qt imported target is
# used in the COVERAGE CMake configuration.
set_target_properties(Qt5::Core PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "RELEASE")


file(GLOB_RECURSE SOURCES
    "*.h"
    "*.cpp"
)

set(UI
    mainwindow.ui
)

add_executable(CAE
    ${SOURCES}
    ${HEADERS}
    ${UI}
)

add_to_cached_list( CGAL_EXECUTABLE_TARGETS CAE )

# OpenMP
# Makes windows build slower for some reason.
#find_package(OpenMP)
#if(OpenMP_CXX_FOUND)
#    target_link_libraries(CAE
#		OpenMP::OpenMP_CXX)
#endif()

# Link the executable to CGAL and third-party libraries
target_link_libraries(CAE 
    ${CGAL_LIBRARIES} 
    ${CGAL_3RD_PARTY_LIBRARIES} )

if (CMAKE_COMPILER_IS_GNUCXX)
    set_target_properties(CAE PROPERTIES COMPILE_FLAGS "-Wall -Wextra")
    set_target_properties(CAE PROPERTIES COMPILE_FLAGS "-fopenmp")
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
	${Boost_INCLUDE_DIR}
	${GLEW_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR}
	${GLUT_INCLUDE_DIR}
)

find_package(OpenGL REQUIRED)

target_link_libraries(CAE 
    Qt5::OpenGL
    Qt5::Widgets
	${Boost_LIBRARIES}
    ${sdl_LIBRARIES}
    ${OPENGL_gl_LIBRARY}
    ${OPENGL_glu_LIBRARY}
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
    ${GLEW_SHARED_LIBRARY_RELEASE}
)

if (WIN32 AND NOT USE_SYSTEM_BOOST)
	# When using the boost from the extern folder, boost libraries are linked this way instead of
	# via Boost_LIBRARIES, see https://stackoverflow.com/a/56036743
	target_link_libraries(CAE
		Boost::atomic
		Boost::chrono
		Boost::date_time
		Boost::filesystem
		Boost::system
		Boost::thread
	)
endif (WIN32 AND NOT USE_SYSTEM_BOOST)

# Installation:
# Copies all binaries of external libraries in the folder that contains the CAE.exe.
# Note: Only needed for Windows because in linux all external libraries are fetched via package management so no need to copy any binaries.
# This also deploys CAE: The resulting folder can be copied on any other system and executed there without the installation of anything else.
# Linux deployment is done via AppImage (see README.md).
if (WIN32)
	set(CMAKE_CAE_INSTALL_DIR_RELEASE ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release/)
	set(CMAKE_CAE_INSTALL_DIR_RELWITHDEBINFO ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/RelWithDebInfo/)
	set(CMAKE_CAE_INSTALL_DIR_DEBUG ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Debug/)

	# Download assets to the binary directory.
	install(CODE "execute_process(\
			COMMAND python -B ${CMAKE_ROOT_SOURCE_DIR}/scripts/python/download_assets.py\
			WORKING_DIRECTORY ${CMAKE_ROOT_BINARY_DIR}/Release)"
			CONFIGURATIONS Release)
	install(CODE "execute_process(\
			COMMAND python -B ${CMAKE_ROOT_SOURCE_DIR}/scripts/python/download_assets.py\
			WORKING_DIRECTORY ${CMAKE_ROOT_BINARY_DIR}/RelWithDebInfo)"
			CONFIGURATIONS RelWithDebInfo)
	install(CODE "execute_process(\
			COMMAND python -B ${CMAKE_ROOT_SOURCE_DIR}/scripts/python/download_assets.py\
			WORKING_DIRECTORY ${CMAKE_ROOT_BINARY_DIR}/Debug)"
			CONFIGURATIONS Debug)

	# install qt
	#add_custom_command(TARGET install
	#	COMMAND ${Qt5_DIR}/../../bin/windeployqt.exe
	#	WORKING_DIRECTORY ${CMAKE_CAE_INSTALL_DIR_RELEASE})
		
	# Is called whenever "INSTALL" project is built.
	install(CODE "execute_process(COMMAND ${Qt5_DIR}/../../../bin/windeployqt.exe CAE.exe WORKING_DIRECTORY ${CMAKE_CAE_INSTALL_DIR_RELEASE})"
			CONFIGURATIONS Release)
	install(CODE "execute_process(COMMAND ${Qt5_DIR}/../../../bin/windeployqt.exe CAE.exe WORKING_DIRECTORY ${CMAKE_CAE_INSTALL_DIR_RELWITHDEBINFO})"
			CONFIGURATIONS RelWithDebInfo)
	install(CODE "execute_process(COMMAND ${Qt5_DIR}/../../../bin/windeployqt.exe CAE.exe WORKING_DIRECTORY ${CMAKE_CAE_INSTALL_DIR_DEBUG})"
			CONFIGURATIONS Debug)

	# Install boost
	function(INSTALL_BOOST_COMPONENT name)
		# Release
		# UTF_BASE_NAME_RELEASE -> name of file (without extension)
		# UTF_BASE_NAME_RELEASE -> path to file
		get_filename_component(UTF_BASE_NAME_RELEASE "${Boost_${name}_LIBRARY_RELEASE}" NAME_WE)
		get_filename_component(UTF_PATH_RELEASE "${Boost_${name}_LIBRARY_RELEASE}" PATH)
		install(FILES ${UTF_PATH_RELEASE}/${UTF_BASE_NAME_RELEASE}.dll
			DESTINATION ${CMAKE_CAE_INSTALL_DIR_RELEASE}
			CONFIGURATIONS Release)
		install(FILES ${UTF_PATH_RELEASE}/${UTF_BASE_NAME_RELEASE}.dll
			DESTINATION ${CMAKE_CAE_INSTALL_DIR_RELWITHDEBINFO}
			CONFIGURATIONS RelWithDebInfo)
		# Debug
		get_filename_component(UTF_BASE_NAME_DEBUG "${Boost_${name}_LIBRARY_DEBUG}" NAME_WE)
		get_filename_component(UTF_PATH_DEBUG "${Boost_${name}_LIBRARY_DEBUG}" PATH)
		install(FILES ${UTF_PATH_DEBUG}/${UTF_BASE_NAME_DEBUG}.dll
			DESTINATION ${CMAKE_CAE_INSTALL_DIR_DEBUG}
			CONFIGURATIONS Debug)
	endfunction(INSTALL_BOOST_COMPONENT)
	INSTALL_BOOST_COMPONENT("SYSTEM")
	INSTALL_BOOST_COMPONENT("THREAD")
	INSTALL_BOOST_COMPONENT("FILESYSTEM")
	INSTALL_BOOST_COMPONENT("DATE_TIME")

	# Install the files for all configurations, i.e. Release, RelWithDebInfo, Debug
	function(CAE_INSTALL_ALL files)
		install(FILES ${files}
			DESTINATION ${CMAKE_CAE_INSTALL_DIR_RELEASE}
			CONFIGURATIONS Release)
		install(FILES ${files}
			DESTINATION ${CMAKE_CAE_INSTALL_DIR_RELWITHDEBINFO}
			CONFIGURATIONS RelWithDebInfo)
		install(FILES ${files}
			DESTINATION ${CMAKE_CAE_INSTALL_DIR_DEBUG}
			CONFIGURATIONS Debug)
	endfunction()

	CAE_INSTALL_ALL(${CMAKE_EXTERN_DIR}/glew/bin/Release/x64/glew32.dll)
	CAE_INSTALL_ALL(${CMAKE_EXTERN_DIR}/freeglut/bin/x64/freeglut.dll)
	CAE_INSTALL_ALL(${CMAKE_EXTERN_DIR}/gmp/lib/libmpfr-4.dll)
	CAE_INSTALL_ALL(${CMAKE_EXTERN_DIR}/gmp/lib/libgmp-10.dll)
		
	#install(TARGETS CAE
	#    DESTINATION ${CMAKE_CAE_INSTALL_DIR_RELEASE})
	#install(TARGETS CAE
	#    DESTINATION ${CMAKE_CAE_INSTALL_DIR_RELWITHDEBINFO})
	#install(TARGETS CAE
	#    DESTINATION ${CMAKE_CAE_INSTALL_DIR_DEBUG})
endif (WIN32)
    

