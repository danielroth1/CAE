#ifndef RIGIDRENDERMODEL_H
#define RIGIDRENDERMODEL_H


#include <data_structures/DataStructures.h>
#include <scene/model/PolygonRenderModel.h>

#include <memory>
#include <vector>


class Polygon;
class Renderer;
class RenderLine;
class RigidBody;

class RigidRenderModel : public PolygonRenderModel
{
public:
    RigidRenderModel(std::shared_ptr<Polygon> polygon,
                     RigidBody* rb);


    // RenderModel interface
public:
    virtual void reset();
    virtual void update();
    virtual void addToRenderer(Renderer* renderer);
    virtual void removeFromRenderer(Renderer* renderer);

private:
    RigidBody* mRigid;

    // Render objects
    std::shared_ptr<RenderLine> mRenderLine;

};

#endif // RIGIDRENDERMODEL_H
