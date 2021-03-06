#ifndef RENDEROBJECT_H
#define RENDEROBJECT_H

#include <GL/glew.h>

#include <data_structures/DataStructures.h>
#include <Eigen/Geometry>

class SceneObject;
class RenderMaterial;
class RenderObjectVisitor;

// RenderObject
// Affine Transformations:
// Has a transformation matrix to transform this objects (Eigen::Affine3f mTransform)
// local coordinate system. Affine transformations (translation, rotation, scaling)
// that affect each vertex equally don't need to be applie to each vertex. It is
// sufficient to adapt this matrix by calling the provided methods in this class:
// translate, rotate, etc. Missing affine transformations can be added any time.
class RenderObject : public std::enable_shared_from_this<RenderObject>
{
public:

    enum DrawMode {
        SURFACE, WIREFRAME
    };

    // make this class independent of SceneObject

    // Standard constructor
    // Visible is set to true and color to white.
    RenderObject();

    // Visitor
    virtual void accept(RenderObjectVisitor& visitor) = 0;

    // Drawing
    virtual void draw() = 0;

    virtual void drawImmediate() = 0;

    virtual void drawArray() = 0;

    virtual void drawVBO() = 0;


    // Handling
    // Updates the vertex positions in the buffer.
    virtual void update() = 0;

    virtual void createBuffers() = 0;

    /*
     * Update position and vertex buffers by copying the
     * data from their respecitve vectors in the buffer.
     */
    virtual void refreshBuffers() = 0;

    virtual void cleanup() = 0;

    virtual void initialize() = 0;

    // Setters/ Getters
    virtual void setDrawMode(DrawMode dm);
    virtual DrawMode getDrawMode();

    std::shared_ptr<RenderMaterial> getRenderMaterial() const;
    void setRenderMaterial(const std::shared_ptr<RenderMaterial>& renderMaterial);

    virtual bool isVisible() const;
    virtual void setVisible(bool visible);

    // This does not work for Polygons. Here use RenderPolygonData.
    bool isWireframeEnabled() const;

    // This does not work for Polygons. Here use RenderPolygonData.
    void setWireframeEnabled(bool wireframeEnabled);

    // Returns a reference to the transformatoin matrix.
    // This method can be used to adapt the transformation
    // matrix using Eigen syntax.
    Eigen::Affine3f& getTransform();

    // Applies a translation to the transformation matrix
    void translate(const Eigen::Vector3d& v);
    void translate(const Eigen::Vector3f& v);

    void rotate(const Eigen::Quaterniond& q);
    void rotate(const Eigen::Quaternionf& q);

    void scale(const Eigen::Vector3d& v);
    void scale(const Eigen::Vector3f& v);

protected:
    virtual ~RenderObject();

    // Enableds/ Disables wireframe according to the set flag. Must be called
    // before rendering the object.
    void setPolygonMode();

    DrawMode mDm;

    std::shared_ptr<RenderMaterial> mRenderMaterial;

    bool mVisible;

    // Transformation matrix
    Eigen::Affine3f mTransform;

    bool mWireframeEnabled;

};

#endif // RENDEROBJECT_H
