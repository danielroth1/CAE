#ifndef TETIMPORTER_H
#define TETIMPORTER_H

#include "SceneImporter.h"


// Importer for .tet files. These files are used to store volumeric 3D Polygon
// data. This is a relatively unknown format but at least it is very simple.
//
// Other info:
// - Supported version: 1.0
// - No support for materials
//
// A tet file looks like this (something within <> is a comment):
//
// tet version 1.0
// num_materials <n> [n is number of materials]
// num_vertices <n> [n is number of vertices]
// num_tetras <n> [n is number of tetrahedrons]
// num_triangles <n> [n is number of triangles]
// MATERIALS
// <list of materials>
// TETRAS
// <list of tetrahedrons>
// TRIANGLES
// <list of triangles>
//
// As an example see the .tet file that describes two tetrahedrons:
// tet version 1.0
// num_materials 0
// num_vertices 5
// num_tetras 2
// num_triangles 0
// MATERIALS
// VERTICES
// 0 0 0 0.000000 1.000000 0.000000 0.000000 0.000000
// 0 0.5 0  0.000000 1.000000 0.000000 0.000000 0.000000
// -0.5 0 0 0.000000 1.000000 0.000000 0.000000 0.000000
// 0 0 0.5 0.000000 1.000000 0.000000 0.000000 0.000000
// 0 0 -0.5 0.000000 1.000000 0.000000 0.000000 0.000000
// TETRAS
// 0 1 2 3 0
// 0 2 1 4 0
//
class TetImporter : public SceneImporter
{
public:
    TetImporter();

    // SceneImporter interface
public:
    virtual std::string getFileFormat() const;
    virtual SGNode* importFile(File file, ApplicationControl* ac);
};

#endif // TETIMPORTER_H
