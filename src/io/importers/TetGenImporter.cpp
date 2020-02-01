#include "TetGenImporter.h"

#include <scene/model/PolygonRenderModel.h>

#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>
#include <scene/data/geometric/Polygon3DTopology.h>
#include <scene/data/geometric/TopologyFactory.h>

#include <ApplicationControl.h>
#include <memory>

#include <io/IOUtils.h>

TetGenImporter::TetGenImporter()
{

}

SGNode* TetGenImporter::importNodeFace(
        const File& nodeFile,
        const File& faceFile,
        ApplicationControl* ac)
{
    Vectors vertices = importNode(nodeFile);
    Faces faces = importFace(faceFile);

    std::shared_ptr<Polygon2D> poly2 =
            std::make_shared<Polygon2D>(vertices, faces);
    return ac->getSGControl()->createLeafNode(nodeFile.getName(), nullptr, poly2);
}

SGNode* TetGenImporter::importNodeEle(
        const File& nodeFile,
        const File& eleFile,
        ApplicationControl* ac)
{
    Vectors vertices = importNode(nodeFile);
    Cells cells = importEle(eleFile);

    std::shared_ptr<Polygon3DTopology> topology =
            TopologyFactory::createPolygon3DTopology(cells, vertices.size());
    std::shared_ptr<Polygon3D> poly3 =
            std::make_shared<Polygon3D>(vertices, topology);
    return ac->getSGControl()->createLeafNode(nodeFile.getName(), nullptr, poly3);
}

SGNode* TetGenImporter::importFiles(
        const std::vector<File>& files, ApplicationControl* ac)
{
    auto itNode = std::find_if(files.begin(), files.end(), [](const File& f)
    {
        return f.getExtension() == ".node";
    });
    auto itFace = std::find_if(files.begin(), files.end(), [](const File& f)
    {
        return f.getExtension() == ".face";
    });
    auto itEle = std::find_if(files.begin(), files.end(), [](const File& f)
    {
        return f.getExtension() == ".ele";
    });

    if (itNode != files.end() && itFace != files.end())
    {
        // (node, face)
        return importNodeFace(*itNode, *itFace, ac);
    }
    else if (itNode != files.end() && itEle != files.end())
    {
        // (node, ele)
        return importNodeEle(*itNode, *itEle, ac);
    }
    return nullptr;
}

std::string TetGenImporter::getFileFormat() const
{
    return "node face ele";
}

SGNode* TetGenImporter::importFile(File /*file*/, ApplicationControl* /*ac*/)
{
    return nullptr;
}

Vectors TetGenImporter::importNode(const File& nodeFile)
{
    Vectors vertices;
    bool successful;
    std::ifstream in = nodeFile.read(successful);
    if (!successful)
    {
        std::cout << "Could not read file: " << nodeFile.getPath() << "\n";
        return vertices;
    }

    std::string line;
    std::getline(in, line);
    std::stringstream ss(line);

    // <num vertices> 3 0 0
    Eigen::Vector4i header = IOUtils::readVector<int, 4>(ss);

    vertices.reserve(header(0));
    while (std::getline(in, line))
    {
        // read line
        std::stringstream ss(line);
        std::string word;

        int index;
        ss >> index;
        vertices.push_back(IOUtils::readVector<double, 3>(ss));
    }
    return vertices;
}

Faces TetGenImporter::importFace(const File& faceFile)
{
    Faces faces;
    bool successful;
    std::ifstream in = faceFile.read(successful);
    if (!successful)
    {
        std::cout << "Could not read file: " << faceFile.getPath() << "\n";
        return faces;
    }

    std::string line;
    std::getline(in, line);
    std::stringstream ss(line);

    // <# of faces> <boundary marker (0 or 1)>
    Eigen::Vector2i header = IOUtils::readVector<int, 2>(ss);

    int firstIndex = -1;
    faces.reserve(header(0));
    while (std::getline(in, line))
    {
        std::stringstream ss(line);
        std::string word;

        if (firstIndex == -1)
        {
            ss >> firstIndex;
        }
        else
        {
            int index;
            ss >> index;
        }
        faces.push_back(IOUtils::readArray<unsigned int, 3>(ss));
        faces.back()[0] -= firstIndex;
        faces.back()[1] -= firstIndex;
        faces.back()[2] -= firstIndex;
    }
    return faces;
}

Cells TetGenImporter::importEle(const File& eleFile)
{
    Cells cells;
    bool successful;
    std::ifstream in = eleFile.read(successful);
    if (!successful)
    {
        std::cout << "Could not read file: " << eleFile.getPath() << "\n";
        return cells;
    }

    std::string line;
    std::getline(in, line);
    std::stringstream ss(line);

    // <# of tetrahedra> <nodes per tetrahedron> <# of attributes>
    Eigen::Vector3i header = IOUtils::readVector<int, 3>(ss);

    cells.reserve(header(0));
    int firstIndex = -1;
    while (std::getline(in, line))
    {
        std::stringstream ss(line);
        std::string word;

        if (firstIndex == -1)
        {
            ss >> firstIndex;
        }
        else
        {
            int index;
            ss >> index;
        }
        cells.push_back(IOUtils::readArray<unsigned int, 4>(ss));
        cells.back()[0] -= firstIndex;
        cells.back()[1] -= firstIndex;
        cells.back()[2] -= firstIndex;
        cells.back()[3] -= firstIndex;
    }
    return cells;
}
