# CAE

CAE is a tool for the simulation of rigid bodies and deformable objects. Geometric meshes can be imported and transformed to a three dimensional representation (made up of tetrahedrons). Those can then be simulated with the Corotated Finite Element Method. Deformables and rigid bodies interact with each other via collisions. Simple spring forces can be applied to vertices of simulated objects.

The main menu of CAE with "Example 2" loaded:
![main2](https://user-images.githubusercontent.com/34305776/55095678-d63c6e80-50b8-11e9-9b58-bcea7c0c389b.png)

Importing a model of an elephant as .off file:
![step1_import](https://user-images.githubusercontent.com/34305776/55096875-fff69500-50ba-11e9-9cba-a3236449d76c.png)

After converting the elephant it is simulated as deformable. Some vertices were selected, truncated, and moved to a different location, inducing elastic deformations:
![step5_simulate](https://user-images.githubusercontent.com/34305776/55096962-2fa59d00-50bb-11e9-9bf0-5a885e127864.png)

## How to use CAE
To simulate a rigid body, simply import the geometric data as .off file and create a rigid body by selecting the node (while having Selection Type: Nodes) and clicking in the right menu Simulation -> Simulate -> Rigid Body.

To create a deformable that is simulated with the corotated FEM, the geometry first needs to be converted to a tree dimensional polygon. To do so, click "Convert" under the "Mesh Converter" tab. The deformable is then created by clicking Simulation -> Simulate -> Deformable. To make either collidable, click the corresponding Simulation -> Simulate -> Collidable.

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
