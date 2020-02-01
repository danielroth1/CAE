#include "OBJExporter.h"

#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon2DTopology.h>
#include <scene/data/geometric/Polygon3D.h>

#include <io/File.h>
#include <io/IOUtils.h>

#include <fstream>
#include <iomanip>

OBJExporter::OBJExporter()
{

}

OBJExporter::~OBJExporter()
{

}

std::string OBJExporter::getFileFormat() const
{
    return ".obj";
}

bool OBJExporter::exportToFilePoly2(
        const File& file,
        const std::shared_ptr<Polygon2D>& poly2)
{
    return exportToFilePoly2(file, poly2->getAccessor2D());
}

bool OBJExporter::exportToFilePoly3(
        const File& file,
        const std::shared_ptr<Polygon3D>& poly3)
{
    return exportToFilePoly2(file, poly3->getAccessor2D());
}

bool OBJExporter::exportToFilePoly2(
        const File& file,
        const std::shared_ptr<Polygon2DAccessor>& accessor)
{
    std::ofstream outfile;
    outfile.open(file.getPath(), std::ios::out | std::ios::trunc );

    outfile << "o " << file.getName() << std::endl;
    for (size_t i = 0; i < accessor->getSize(); ++i)
    {
        outfile << "v ";
        IOUtils::writeVector<double, 3>(outfile, accessor->getPosition(i)) << std::endl;
    }

    for (size_t i = 0; i < accessor->getSize(); ++i)
    {
        outfile << "vn ";
        IOUtils::writeVector<double, 3>(outfile, accessor->getVertexNormals()[i]) << std::endl;
    }

    Polygon2DTopology& topo2 = accessor->getTopology2D();
    for (size_t i = 0; i < topo2.getFacesIndices().size(); ++i)
    {
        //   <v>/<vt>/<vn> <v>/<vt>/<vn> <v>/<vt>/<vn>
        Face& f = topo2.getFacesIndices()[i];
        outfile << "f ";
        for (size_t j = 0; j < 3; ++j)
        {
            IOUtils::writeNumber<int>(outfile, f[j] + 1);
            outfile << "//"; // no texture support yet
            IOUtils::writeNumber<int>(outfile, f[j] + 1);
            if (j != 2)
                outfile << " ";
        }
        if (i != topo2.getFacesIndices().size() - 1)
            outfile << std::endl;
    }

    return true;
}
