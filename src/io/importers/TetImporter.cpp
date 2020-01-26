#include "TetImporter.h"

#include <scene/model/PolygonRenderModel.h>

#include <scene/data/geometric/Polygon3D.h>
#include <scene/data/geometric/Polygon3DTopology.h>
#include <scene/data/geometric/TopologyFactory.h>

#include <ApplicationControl.h>
#include <memory>

#include <io/IOUtils.h>

TetImporter::TetImporter()
{

}

std::string TetImporter::getFileFormat() const
{
    return "tet";
}

SGNode* TetImporter::importFile(File file, ApplicationControl* ac)
{
    Vectors vertices;
    Vectors vertexNormals;
    Faces faces;
    Faces outerFaces;
    Cells cells;

    // Steps:
    // 1. Gather data and store it in:
    //    vertices, vertexNormals, faces, outerFaces, cells
    // 2. from those, create SGNode, Polygon3D and auxilary classes

    // Step 1)
    bool successful;
    std::ifstream in = file.read(successful);
    if (!successful)
    {
        std::cout << "Could not read file: " << file.getPath() << "\n";
        return nullptr;
    }

    int state = 0;
    std::string line;
    while (std::getline(in, line))
    {
        // read line
        std::stringstream ss(line);
        std::string word;

        // reading state
        // 0 - none
        // 1 - MATERIALS
        // 2 - VERTICES
        // 3 - TETRAS
        // 4 - TRIANGLES
        size_t counter;
        size_t numElements;

        // version
        if (state != 0)
        {
            switch(state)
            {
            case 1:
                // materials not supported yet
                break;
            case 2:
                vertices[counter] = IOUtils::readVector<double, 3>(ss);
                break;
            case 3:
                cells[counter] = IOUtils::readArray<unsigned int, 4>(ss);
                break;
            case 4:
                faces[counter] = IOUtils::readArray<unsigned int, 3>(ss);
                break;
            }
            ++counter;
        }
        else
        {
            ss >> word;

            if (word == "tet")
            {
                ss >> word;
                if (word == "version")
                {
                    ss >> word;
                    if (word != "1.0")
                    {
                    std::cout << "The file is of version " << word << " which is"
                              << " not supported. The supported version is 1.0."
                              << "The model might have errors.";
                    }
                }
            }
            else if (word == "num_materials")
            {
                // materials not supported yet
            }
            else if (word == "num_vertices")
            {
                vertices.resize(IOUtils::readNumber<size_t>(ss));
            }
            else if (word == "num_tetras")
            {
                cells.resize(IOUtils::readNumber<size_t>(ss));
            }
            else if (word == "num_triangles")
            {
                faces.resize(IOUtils::readNumber<size_t>(ss));
            }
            else if (word == "MATERIALS")
            {
                state = 1;
                counter = 0;
                numElements = 0;
            }
            else if (word == "VERTICES")
            {
                state = 2;
                counter = 0;
                numElements = vertices.size();
            }
            else if (word == "TETRAS")
            {
                state = 3;
                counter = 0;
                numElements = cells.size();
            }
            else if (word == "TRIANGLES")
            {
                state = 4;
                counter = 0;
                numElements = faces.size();
            }
        }

        if (state != 0 && counter >= numElements)
            state = 0;
    }

    std::shared_ptr<Polygon3DTopology> topology;

    if (faces.empty())
    {
        topology = TopologyFactory::createPolygon3DTopology(
                    cells, vertices.size());
    }
    else
    {
        topology = TopologyFactory::createPolygon3DTopologyWithOuter(
                    faces, cells, vertices.size());
    }

    // Step 2)
    std::shared_ptr<Polygon3D> poly3 =
            std::make_shared<Polygon3D>(vertices, topology);
    return ac->getSGControl()->createLeafNode(file.getName(), nullptr, poly3);
}
