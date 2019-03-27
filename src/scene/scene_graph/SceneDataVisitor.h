#ifndef SCENEDATAVISITOR_H
#define SCENEDATAVISITOR_H

class SceneData;
class SceneLeafData;

class SceneDataVisitor
{
public:
    virtual void visit(SceneData* sceneData) = 0;
    virtual void visit(SceneLeafData* sceneData) = 0;

protected:
    SceneDataVisitor();
    virtual ~SceneDataVisitor();
};

#endif // SCENEDATAVISITOR_H
