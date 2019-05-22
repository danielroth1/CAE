#ifndef SGUICONTROL_H
#define SGUICONTROL_H


class RenderModelVisitor;
class SGControl;
class UIControl;

class SGUIControl
{
public:
    SGUIControl();

    void initialize(SGControl* sgControl, UIControl* uiControl);

    // Traverses the scene graph and enables the visualization of
    // face normals for all render models that support this.
    void setVisualizeFaceNormals(bool visualizeFaceNormals);
    bool isVisualizeFaceNormals() const;

    void setVisualizeVertexNormals(bool visualizeVertexNormals);
    bool isVisualizeVertexNormals() const;

private:

    void iterateRenderModels(RenderModelVisitor& renderModelVisitor);

    // Control classes
        SGControl* mSgControl;
        UIControl* mUiControl;

    // Regular members
        bool mVisualizeFaceNormals;
        bool mVisualizeVertexNormals;
};

#endif // SGUICONTROL_H
