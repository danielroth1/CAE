#ifndef RENDERMODEL_H
#define RENDERMODEL_H

#include <memory>

class RenderModelVisitor;
class RenderObject;
class Renderer;

// One must override setVisible(bool) and set the corresponding
// RenderObjects that are managed by the renderModel visible
// How the communication between RenderModel and RenderObjects
// should be done:
// RenderModel creates the RenderObjects and stores a shared
//      pointer to them.
// RenderModel calls the getters of RenderObjects to obtain
//      Monitors that point to the data, the RenderModel
//      wants to adapt in the update() method.
// RenderModel updates data with Monitors. Uses monitors
//      lock() method to guarantee a thread safe access
//      of the data.
class RenderModel : public std::enable_shared_from_this<RenderModel>
{
public:
    // Reset data structures.
    // Call this method when the size of the
    // containers changed. Updates the changed
    // values as well so a call of update() is
    // not necessary.
    virtual void reset() = 0;

    // Updates data structures.
    // Call this method when values in the
    // data structures changed. If the data
    // structures changed in size, call reset()
    // instead.
    virtual void update() = 0;

    // Revalidates the render model. Call this method when
    // there are structural changes in the data, i.e.
    // - number of positions changed
    // - position representation type changed
    virtual void revalidate() = 0;

    virtual void accept(RenderModelVisitor& v) = 0;

    // Render Access Methods
    // Adds all necessary RenderObjects to renderer.
    virtual void addToRenderer(Renderer* renderer) = 0;
    // Removes all used RenderObjects from renderer.
    virtual void removeFromRenderer(Renderer* renderer) = 0;

    bool isAddedToRenderer();
    void setAddedToRenderer(bool addedToRenderer);

    // if true, the update method is called in each draw step
    // independently of any other condition. Only set this
    // true for lightweight objects for which it is
    // difficulty to determine if an update is necessary,
    // e.g. when the geometric data contains references
    // (GeometricPointRef, ...)
    bool isAlwaysUpdate();
    void setAlwaysUpdate(bool alwaysUpdate);

    virtual bool isVisible() const;
    virtual void setVisible(bool visible);

    virtual bool isWireframeEnabled() const;
    virtual void setWireframeEnabled(bool wireframeEnabled);

    virtual ~RenderModel();
protected:
    RenderModel();

    bool mAddedToRenderer;
    bool mAlwaysUpdate;

    bool mVisible;

private:
    bool mWireframeEnabled;
};

#endif // RENDERMODEL_H
