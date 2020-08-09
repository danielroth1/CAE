# CAE

CAE (Computer Aided Engineering) is a tool for the simulation of collidable rigid and deformable bodies.
By separating collision and deformation geometries, a highly performant but yet realistic simulation of deformable bodies is possible.
The process from importing a mesh to simulating them only takes a few mouse clicks.
For deformables where a complex mesh discretization is needed, a powerfull but easy to use mesh conversion tool is provided.
The simulation of deformables is to a certain degree realistic thanks to the use of the Corotated Finite Element Method.
Visual pleasing deformations are achieved with mesh interpolators that connect low resolution simulation meshes to high resolution visual ones.

Video of CAEs Car Crash demo: https://youtu.be/xlMe7Q1j4Lc

<img src="https://user-images.githubusercontent.com/34305776/89671867-04a09f80-d8e4-11ea-878d-de071784d1a1.png" width="1000"/>

## Features
- Rigid bodies
- Joints
- Deformables (Corotated FEM)
- Collisions between rigids and deformables
- Mesh-Converter (CGAL)
- Mesh-Interpolators
- Separate deformation, collision, and visualization mesh for highly performant simulation
- Importers: obj, off, tet, TetGen
- Exporters: obj
- Renderer (OpenGL based)
- Templated tree data structure
- Easily extend the UI with Qt member widgets
- Multithreading concepts (Monitors and Domains)

## Build Instructions
Currently, only Linux is supported.

To build with CMake, see "Building with CMake" below. I am using qmake because of its better integration with QtCreator. If you want to use it as well, see "Building with qmake".

### Required Libraries
- Boost (>= 1.57)
- CGAL (>= 5.0)
- CMake (>= 3.0)
- Eigen3 (>= 3.2.10)
- GCC (>= 5.0)
- GLEW
- GLU/OpenGL
- GLUT
- Qt5 (>= 5.4)

Install packages on Linux using apt:
```
sudo apt-get install unzip git cmake
sudo apt-get install build-essential qt5-default libglew-dev freeglut3-dev libboost-all-dev libmpfr-dev libgmp3-dev libxmu-dev libxi-dev
```
Eigen and CGAL don't need to be installed because they are header-only. Instead, they are automatically downloaded when running qmake oder cmake.

### Building with CMake
Install/ build the required libraries. If qt was manually installed, set Qt5_DIR to \<path-to-QT\>/\<version\>/gcc_64/lib/cmake/Qt5
Then run:
```
mkdir CAE && cd CAE
git clone https://github.com/danielroth1/CAE.git
mkdir build-CAE-Release && cd build-CAE-Release
cmake -DCMAKE_BUILD_TYPE=Release ../CAE
make -j 8
```
To build in Debug mode: replace "Release" with "Debug"

### Building with qmake
With QtCreator:
```
mkdir CAE && cd CAE
git clone git@github.com:danielroth1/CAE.git
```
open project in QtCreator and select the CAE.pro

Without QtCreator:
```
mkdir CAE && cd CAE
git clone https://github.com/danielroth1/CAE.git
mkdir build-CAE-Release && cd build-CAE-Release
qmake ../CAE/CAE.pro CONFIG+=release
make -j 8
```
To build in Debug mode: replace "release" with "debug"

### Deployment
It's possible to create an AppImage which can be executed on other linux based distros that don't have any of the above libraries installed (uses [linuxdeployqt](https://github.com/probonopd/linuxdeployqt)). To do so, first, install the required qt lib:
```
sudo apt-get install qttools5-dev-tools
```
and then execute from the CAE directory:
```
cd deployment
bash create_appimage.sh ../../build-CAE-Release
```
This generates an AppImage file:\
deployment/appimage/CAE-\<commit-SHA1-ID\>-x86_64.AppImage

#### Building and Deploying with Ubuntu-14.04.6
It is recommened to execute the deployment on the oldest supported Ubuntu LTS version to ensure compatibility for older Linux distros (see [A not on binary compatibility](https://github.com/probonopd/linuxdeployqt#a-note-on-binary-compatibility) ).
As of this writing, the oldest LTS version is Ubuntu-14.04.6 which is supported until April 2022 (https://wiki.ubuntu.com/Releases).
Many of the libraries from the official package sources of Ubuntu-14.04.6 are too old.
It follows step by step instructions on what needs to be done to install newer packages and successfully build CAE on Ubuntu-14.04.6.

Install g++5 and set as standard:
```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-5 g++-5
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5
```

Install newer cmake 3.2:
```
sudo apt-get install software-properties-common
sudo add-apt-repository ppa:george-edison55/cmake-3.x
sudo apt-get update
```

Build and install boost 1.57.0: (./b2 can take a couple of minutes)
```
cd ~
sudo apt-get autoremove package libboost-all-dev
sudo apt-get install build-essential g++ python-dev autotools-dev libicu-dev build-essential libbz2-dev 
wget http://downloads.sourceforge.net/project/boost/boost/1.57.0/boost_1_57_0.tar.bz2
tar xjf ./boost_1_57_0.tar.bz2 && cd boost_1_57_0
./bootstrap.sh --prefix=/usr/
sudo ./b2 -j 8
sudo ./b2 install
```

Install Qt 5.4.2:
```
sudo add-apt-repository ppa:beineri/opt-qt542-trusty
sudo apt-get update
sudo apt install qt54-meta-full
mkdir -p ~/.config/qtchooser/
printf "/opt/qt54/bin\n/opt/qt54/lib" > ~/.config/qtchooser/default.conf
```

Build CAE:
```
cd ~
mkdir CAE && cd CAE
git clone https://github.com/danielroth1/CAE.git
mkdir build-CAE-Release && cd build-CAE-Release
cmake -DCMAKE_BUILD_TYPE=Release -DQt5_DIR=/opt/qt54/lib/cmake/Qt5 -DQt5Widgets_DIR=/opt/qt54/lib/cmake/Qt5Widgets -DQt5OpenGL_DIR=/opt/qt54/lib/cmake/Qt5OpenGL -DQt5Core_DIR=/opt/qt54/lib/cmake/Qt5Gui -DQt5Core_DIR=/opt/qt54/lib/cmake/Qt5Core ../CAE
make -j 8
```

Create the AppImage:
```
cd deployment
bash create_appimage.sh ../../build-CAE-Release
```
The resulting AppImage will run on most of the linux distros out there out of the box.

## Controls
| Action                      | key               |
|-----------------------------|-------------------|
| camera rotation             | mouse left click  |
| camera translation          | w, a, s, d        |
| select, move, apply force   | mouse right click |

## Documentation
The following paragraphs describe the implemented approaches a bit more in detail. Paper references are under _Bibliography_ at the bottom of this document.

### Rigids
Rigid bodies can be simulated extremely efficiently because each rigid is internally only described by a single position (center of mass) and a rotation.
The impulse based approach by Bender et al. is implemented to handle **collisions** [1] and variouse type of **joints** [2].
The following shows a box with four rotating tires that are connected with springs. Line-joints fixate each tire along its y-axis while double-axis-rotational-joints make sure that the tires only rotate forwards.

<img src="https://user-images.githubusercontent.com/34305776/89443293-616b5100-d750-11ea-93ae-b31cd564ffcb.png" width="300"/>

### Deformables
Deformables are simulated with the so called **Corotational Finite Element Method** [3]. Implicit time integration is used to calculate velocities and positions of each vertex in each time step. This has two effects: It makes the simulation unconditionally stable even for higher time step sizes and the required simulation time in each time step is constant which is beneficial in real time applications where a certain frame rate must be ensured.

### Co-Simulation
Deformables and rigids interact with each other in a Co-Simulation environment. This means that the two solvers (impulse based for rigids and FEM for deformables) that usually operate independently are communicating with each other via forces. In consequence the simulation of collisions between rigids and deformables is possible.

### Mesh-Converter
To be able to deform any given triangle mesh, a Mesh-Converter is provided which creates a tetrahedron mesh by discretizing the by the triangle mesh surrounded volume.
The converter uses an [algorithm](https://doc.cgal.org/latest/Mesh_3/index.html#title10) from the CGAL library.
It's possible to influence quality and resolution of the generated mesh by setting various parameters.
It's recommended to first try different values for "facet_distance" and then set the other parameters to fine tune.

<img src="https://user-images.githubusercontent.com/34305776/65803792-fa6c2b80-e17f-11e9-945e-8e51d1b1f258.jpg" width="300"/>

<em>Low resolution grey mesh converted from the high resolution red mesh by using a facet_distance of 0.02 and all other values zero. The grey mesh consists of tetrahedrons but only the outer triangles are visualized here.</em>

<img src="https://user-images.githubusercontent.com/34305776/89728930-4559ff00-da31-11ea-8235-31f411cfbd04.png" width="300"/>

<em>Demo: InterpolationMeshMesh (Astronaut) [[source](https://nasa3d.arc.nasa.gov/detail/aces)]</em>

### Interpolators
The simulation of deformables is considerably more expensive than of rigids, especially for more detailed tetrahedron meshes. On the other hand, low resolved meshes can be very unplesant to look at. To solve this problem, it is possible to attach a highly detailed mesh on the simulated low resolution discretized mesh. Two of such mesh interpolation methods are implemented:
- The **MeshMeshInterpolator** [4] maps each point of the high resolved mesh w.r.t. the triangles of the outer simulated mesh. This method is best used if there are many vertices of the high resolved mesh that lie outside the simlated mesh.
- The **FEMInterpolator** maps the points of the high resolved mesh w.r.t. the finite elements. This method is better suited for vertices that lie inside the simulated mesh (each vertex is inside a finite element/ tetrahedron).

See FallingObjectsDemo.cpp to see how to use them. For the previous Armadillo a mesh-mesh interpolation would be ideal.

The following example shows how each vertex of the high resolved sphere is mapped on the triangles of a low resolved cube. The second picture shows how each vertex of the sphere is interpolated when the cube deforms. Even in such an extrem scenario where both meshes have nothing in common, the interpolation still looks plausible.

<img src="https://user-images.githubusercontent.com/34305776/65803872-43bc7b00-e180-11e9-9c8b-775df654d6c8.png" width="500"/>

### Collisions
Collision handling works by resolving vertex-face and edge-edge collisions. Impulses are applied to fulfill collision constraints according to Benders approach [1]. To reduce the number of collision checks, an AABB bounding volume hierarchy is used.

Deformable boxes colliding with each other:

<img src="https://user-images.githubusercontent.com/34305776/89667413-63fab180-d8dc-11ea-82c6-830168f8ac2d.png" width="300"/>

Different layers of the AABB bounding volume hierarchy:

<img src="https://user-images.githubusercontent.com/34305776/89669335-9063fd00-d8df-11ea-9766-53e94f308948.png" width="500"/>

CAE offers the possiblity to separate deformation, collision, and visualization meshes.
Collision and visualization mesh are interpolated from the tetrahedron mesh and moved accordingly.
Low resolution deformation meshes are usually sufficient for a reasonable deformation behavior and are a lot faster to simulate.
Collision detection can be based on a low resolution triangle mesh, like the convex hull of the visualized mesh.

<img src="https://user-images.githubusercontent.com/34305776/89727922-4f77ff80-da29-11ea-887a-7b31f0a931ae.png" width="500"/>

<em>A car (from the [FEMFX demo](https://github.com/GPUOpen-Effects/FEMFX/tree/master/samples/FEMFXViewer/models)) that drives into a wall and deforms. 30 of those can be simulated in real time by using different meshes for deformation (green), collision detection (white), and visualization (red).</em>

### I/O
Importers for the 2D mesh file formats .obj and .off as well as 3D file format .tet are implemented. It's possible to export in .obj format.

<table class="tg">
  <tr>
    <th class="tg-031e"></th>
    <th class="tg-s6z2" colspan="2">2D Mesh<br></th>
    <th class="tg-s6z2" colspan="2">3D Mesh<br></th>
  </tr>
  <tr>
    <td class="tg-031e"></td>
    <td class="tg-s6z2">.obj</td>
    <td class="tg-s6z2">.off</td>
    <td class="tg-s6z2">TetGen (.node .face .ele)<br></td>
    <td class="tg-s6z2">.tet</td>
  </tr>
  <tr>
    <td class="tg-031e">Importer</td>
    <td class="tg-s6z2">x</td>
    <td class="tg-s6z2">x</td>
    <td class="tg-s6z2">x</td>
    <td class="tg-s6z2">x</td>
  </tr>
  <tr>
    <td class="tg-031e">Exporter</td>
    <td class="tg-s6z2">x</td>
    <td class="tg-s6z2"></td>
    <td class="tg-s6z2"></td>
    <td class="tg-s6z2"></td>
  </tr>
</table>

### Renderer
CAE uses its own OpenGL based renderer. It supports simple lighting and texturing. Its designed to run in its own thread and, therefore, offers a completely threadsafe access to its rendering components. In CAE the separation between logic and renderer is clearly defined. The communicaiton between them is mostly done in so called RenderModels. This would allow to easily change the underlying rendering engine if necessary.

### Miscellaneous
A **templated tree data structure** is used to define the scene graph and bounding volume hierarchy, see SGCore.h and BVHCore. It is generic enough that it can be used in various other scenarios.

**Qt member widgets** are used to easily adapt class members of common types like bools, doubles, vectors, etc. in the UI, a templated data structure and a corresponding Qt Widget are used. Extending the UI by a check box, number field, and so on for any kind of member only requires a single line of code where either the members memory adress or its getters and setters are provided. See for example SimulationUIControl::init where ui elements are added that change global and object specific simulation parameters. The object specific elements are hidden depending on what object type is selected (e.g. only show FEM material parameters if a deformables is selected).

**Thread safety** is important in a multi threaded environment like a CAD/CAE tool. Usually simulation, renderer, and application all run in their own thread. Special measurements must be applied to avoid race conditions:
- **Monitors**: A Monitor makes the access to any type of data structure threadsafe by using a mutex. If used correctly, common issues that can occur with mutices like deadlocks are avoided. CAE uses Monitors for example in the renderer.
- **Domains**: Often it is necessary to execute methods asynchrously in a designated thread at a point when it is actually safe to execute them (e.g. when changing simulation parameters, the current simulation step should be finished first). CAE achieves this by using Domains. A Domain executes a loop in a thread and at the start of each iteration all operations that are stored in the domains operation queue are executed. A opeartion queue is a data structure that stores method calls and their parameters. By adding a method call to a operation queue of a domain, it will be executed at a later point in the domains thread. Instead of manually creating operations and having to handle function pointers, a macro is used that hides most of the complexity. The only thing that needs to be done is redeclaring every method that should be callabe asynchronously in the same header using the macro. The asynchronouse method is then called by using the classes proxy object \<class_name\>Proxy. For an example to see how this is done see the bottom of SimulationControl.h.


### Bibliography
[1] Bender, Jan, and Alfred Schmitt. "Constraint-based collision and contact handling using impulses." Proceedings of the 19th international conference on computer animation and social agents. 2006.

[2] Bender, Jan, and Alfred A. Schmitt. "Fast Dynamic Simulation of Multi-Body Systems Using Impulses." VRIPhys. 2006.

[3] Müller, Matthias, and Markus Gross. "Interactive virtual materials." Proceedings of Graphics Interface 2004. Canadian Human-Computer Communications Society, 2004.

[4] Kobbelt, Leif, Jens Vorsatz, and Hans-Peter Seidel. "Multiresolution hierarchies on unstructured triangle meshes." Computational Geometry 14.1-3 (1999): 5-24.

