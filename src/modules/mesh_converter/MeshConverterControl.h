#ifndef MESHCONVERTERCONTROL_H
#define MESHCONVERTERCONTROL_H

#include <ui/selection/SelectionListener.h>


class ApplicationControl;
class GeometricData;
class MeshConverter;
class MeshConverterModule;
class Polygon;
class Polygon2D;

// Provides functionality to convert existing meshes
// How this class works:
// There is always a loaded GeometricPolygon2D. This geometry is translatet
// to a CGAL data structure in loadGeometry2D().
// convert() converts this geometry to GeometricPolygon3D.
// By pre loading the 2D geometry to a CGAL polygon, a good performance
// can be achieved because CGAL polygon doesn't need be recreated each time.
class MeshConverterControl
{
public:
    MeshConverterControl(
            MeshConverterModule* module,
            ApplicationControl* ac);

    virtual ~MeshConverterControl();

    void init();

    // Prepares a Polygon2D to be converted to a Polygon3D.
    // Call this method before calling convert().
    void loadGeometry2D(Polygon2D* poly2);

    // Converts the Polygon2D that was loaded in loadGeometry2D.
    //TODO: call these from ui control
    void convert(
            double facetAngle,
            double facetSize,
            double facetDistance,
            double cellSize,
            double cellRadiusEdgeRatio,
            bool renderOnlyOuterFaces = true);

    void revert();

private:

    // Gathers information about geometry and visualizes it.
    void processGeometricData(GeometricData* gd);

    std::vector<
        std::tuple<
            std::shared_ptr<SceneLeafData>,
            std::shared_ptr<Polygon>>> mSavedPolygons;

    ApplicationControl* mAc;
    MeshConverterModule* mModule;

};

#endif // MESHCONVERTERCONTROL_H
