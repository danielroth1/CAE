#ifndef SCENEEXPORTER_H
#define SCENEEXPORTER_H

#include <scene/scene_graph/SGCore.h>

class ApplicationControl;
class File;
class GeometricData;
class GeometricPoint;
class Polygon2D;
class Polygon3D;

class SceneExporter
{
public:

    virtual std::string getFileFormat() const = 0;

    // Exports the given geometricData to the file.
    // \return true if successful.
    bool exportToFile(const File& file,
                      const std::shared_ptr<GeometricData>& gd);

protected:

    SceneExporter();
    virtual ~SceneExporter();

    // Dispatcher method for Polygon2D
    virtual bool exportToFilePoly2(const File& file,
                      const std::shared_ptr<Polygon2D>& poly2);

    // Dispatcher method for Polygon3D
    virtual bool exportToFilePoly3(const File& file,
                      const std::shared_ptr<Polygon3D>& poly3);

    // Dispatcher method for GeometricPoint
    virtual bool exportToFileGp(const File& file,
                      const std::shared_ptr<GeometricPoint>& gp);

private:

};

#endif // SCENEEXPORTER_H
