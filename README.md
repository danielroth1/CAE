CAE is a tool for the simulation of [Rigid Bodies](https://en.wikipedia.org/wiki/Rigid_body) and deformable objects.
Geometric meshes can be imported and converted into a three-dimensional representation (consisting of tetrahedra). These can then be simulated using the Corotated Finite Element Method. Deformable and rigid bodies interact via collisions. Simple spring forces can be applied to vertices of simulated objects.

#### The main menu of CAE  with *Example 2* was loaded in: 
![main2](https://user-images.githubusercontent.com/34305776/55095678-d63c6e80-50b8-11e9-9b58-bcea7c0c389b.png)

#### An Elephant model can be imported, by loading the *as.off* file:
![step1_import](https://user-images.githubusercontent.com/34305776/55096875-fff69500-50ba-11e9-9cba-a3236449d76c.png)


#### After the transformation of the elephant its deformation is simulated. Some nodes were selected, cut and moved to a different location, resulting in elastic deformations:
![step5_simulate](https://user-images.githubusercontent.com/34305776/55096962-2fa59d00-50bb-11e9-9bf0-5a885e127864.png)

#### For collision detection, spheres are used that surround each object. A bounding volume hierarchy is used for efficient access. Layer by layer can be visualized or as in this example all sheets can be visualized at once:
![collision_spheres](https://user-images.githubusercontent.com/34305776/55101329-27059480-50c4-11e9-8083-326ad81e4db9.png)


## How to use CAE
To simulate a rigid body, import the geometry data from the  **.off** file and create a rigid body by selecting the node (while having Selection Type set to "Nodes") than click the right menu: Simulation -> Simulate -> Rigid Body.

To create a deformable that is simulated with the [corotated FEM](https://www.google.com/search?q=corotated+FEM&ie=utf-8&oe=utf-8) , the geometry first needs to be converted to a tree dimensional polygon. To do so by clicking on "Convert" (in the "Mesh Converter" tab). The deformable is then created by clicking Simulation -> Simulate -> Deformable. To make either collidable, click the corresponding Simulation -> Simulate -> Collidable.

## Installation
I am using qmake because of its better integration with QtCreator. If you want to use it as well, see "Building with qmake and QtCreator" (a few sections below).
Despite that I make use of QMake, building with CMake is also perfectly fine.

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
