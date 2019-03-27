# CAE

## Installation
I am using qmake because of its better integration with QtCreator. If you want as well, see "Building with qmake and QtCreator".
Despite that, building with CMake is also perfectly fine.

### Required libraries
* Boost
* CGAL
* Eigen3
* GLEW
* GLU/OpenGL
* GLUT
* Qt5

### Building with CMake:

Install/ build the reqiured libraries. Execute cmake-gui and fill in the missing paths.
For Qt5_DIR use e.g. <path-to-QT>/5.9.1/gcc_64/lib/cmake/Qt5
For CGAL_DIR the path to the CGAL directory is sufficient.

### Building with qmake and QtCreator:

edit the _CAE.pro file and replace <path-to-cgal> with the path to your CGAL directory.
Change the EIGEN_INCLUDE_PATH variable with the path to your eigen include directory.
Finally, rename "_CAE.pro" to "CAE.pro".
Start QtCreator, open new project, and select the CAE.pro.
