
add_definitions( -DBOOST_PARAMETER_MAX_ARITY=12 )


# find required packages
#find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL CONFIG)
find_package(Eigen3 REQUIRED)
find_package(GLUT REQUIRED)

#CGAL BEGIN------------------------------
# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()

# include helper file
include( ${CGAL_USE_FILE} )

#CGAL END------------------------------

# Boost and its components
FIND_PACKAGE( Boost COMPONENTS system thread filesystem REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )


# GLEW
find_package(GLEW REQUIRED)

find_package(Qt5Core)

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
	${GLEW_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR}
	${GLUT_INCLUDE_DIRS}
)

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()


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
    ${GLEW_LIBRARY}
)

