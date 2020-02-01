#include "SceneExporter.h"

#include <scene/data/GeometricDataVisitor.h>

#include <scene/data/geometric/GeometricPoint.h>
#include <scene/data/geometric/Polygon2D.h>
#include <scene/data/geometric/Polygon3D.h>

#include <io/File.h>

bool SceneExporter::exportToFile(const File& file,
                                 const std::shared_ptr<GeometricData>& gd)
{
    class GeometricDataDispatcher : public GeometricDataVisitor
    {
    public:
        GeometricDataDispatcher(SceneExporter& _exporter,
                                const File& _file)
            : exporter(_exporter)
            , file(_file)
        {

        }

        virtual void visit(Polygon2D& polygon2D)
        {
            std::shared_ptr<Polygon2D> poly2 =
                    std::static_pointer_cast<Polygon2D>(polygon2D.shared_from_this());
            returnValue = exporter.exportToFilePoly2(file, poly2);
        }
        virtual void visit(Polygon3D& polygon3D)
        {
            std::shared_ptr<Polygon3D> poly3 =
                    std::static_pointer_cast<Polygon3D>(polygon3D.shared_from_this());
            returnValue = exporter.exportToFilePoly3(file, poly3);
        }
        virtual void visit(GeometricPoint& point)
        {
            std::shared_ptr<GeometricPoint> gp =
                    std::static_pointer_cast<GeometricPoint>(point.shared_from_this());
            returnValue = exporter.exportToFileGp(file, gp);
        }

        bool returnValue;
    private:
        SceneExporter& exporter;
        const File& file;
    } visitor(*this, file);
    gd->accept(visitor);
    return visitor.returnValue;
}

SceneExporter::SceneExporter()
{

}

SceneExporter::~SceneExporter()
{

}

bool SceneExporter::exportToFilePoly2(
        const File& file,
        const std::shared_ptr<Polygon2D>& /*poly2*/)
{
    std::cout << "Can not export file: " << file.getPath() << ".\n";
    std::cout << "Reason: Unsupported geometry type Polygon2D for files of format "
              << getFileFormat() << ".\n";
    return false;
}

bool SceneExporter::exportToFilePoly3(
        const File& file,
        const std::shared_ptr<Polygon3D>& /*poly3*/)
{
    std::cout << "Can not export file: " << file.getPath() << ".\n";
    std::cout << "Reason: Unsupported geometry type Polygon3D for files of format "
              << getFileFormat() << ".\n";
    return false;
}

bool SceneExporter::exportToFileGp(
        const File& file,
        const std::shared_ptr<GeometricPoint>& /*gp*/)
{
    std::cout << "Can not export file: " << file.getPath() << ".\n";
    std::cout << "Reason: Unsupported geometry type GeometricPoint for files of format "
              << getFileFormat() << ".\n";
    return false;
}
