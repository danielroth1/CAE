#ifndef OBJEXPORTER_H
#define OBJEXPORTER_H

#include "SceneExporter.h"

class Polygon2DAccessor;

// Exporter to export Polygon2D and Polygon3D objects in the .obj file format.
// Currently only supports the export of topological and geometric information.
// Does not export any texture or material data.
class OBJExporter : public SceneExporter
{
public:
    OBJExporter();
    virtual ~OBJExporter();

    // SceneExporter interface
public:
    std::string getFileFormat() const override;

    // SceneExporter interface
protected:
    bool exportToFilePoly2(
            const File& file,
            const std::shared_ptr<Polygon2D>& poly2) override;

    bool exportToFilePoly3(
            const File& file,
            const std::shared_ptr<Polygon3D>& poly3) override;

private:
    bool exportToFilePoly2(
            const File& file,
            const std::shared_ptr<Polygon2DAccessor>& accessor);
};

#endif // OBJEXPORTER_H
