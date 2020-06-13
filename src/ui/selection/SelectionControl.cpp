#include "SelectionControl.h"
#include "Selection.h"
#include "SelectionRectangle.h"
#include "SelectionRectangleModel.h"
#include "SelectionSceneData.h"
#include "SelectionSceneDataModel.h"
#include "SelectionVertices.h"
#include "SelectionVerticesModel.h"
#include "SelectionListener.h"

#include <rendering/ViewFrustum.h>
#include <ApplicationControl.h>
#include <scene/data/GeometricData.h>
#include <scene/scene_graph/SGTraverserFactory.h>

#include <iostream>

SelectionControl::SelectionControl(
        ApplicationControl* ac,
        ViewFrustum& viewFrustum)
{
    mAc = ac;

    mSelectionRectangle =
            std::make_unique<SelectionRectangle>();
    mSelectionRectangleModel =
            std::make_unique<SelectionRectangleModel>(
                *mSelectionRectangle.get(),
                &viewFrustum);

    mSelectionVertices =
            std::make_unique<SelectionVertices>();
    mSelectionVerticesModel =
            std::make_unique<SelectionVerticesModel>(*mSelectionVertices.get());

    mSelectionSceneData =
            std::make_unique<SelectionSceneData>();
    mSelectionSceneDataModel =
            std::make_unique<SelectionSceneDataModel>(*mSelectionSceneData.get());

    mSelectionType = UNDEFINED;
    changeSelectionType(SELECT_VERTICES);
    mSelectionMode = RECTANGLE;
}

void SelectionControl::init(Renderer* renderer)
{
    mSelectionSceneDataModel->addToRenderer(renderer);
    mSelectionVerticesModel->addToRenderer(renderer);
    mSelectionRectangleModel->addToRenderer(renderer);
}

void SelectionControl::changeSelectionType(SelectionControl::SelectionType type)
{
    if (mSelectionType == type)
        return;
    mSelectionType = type;

    switch(type)
    {
    case SELECT_SCENE_NODES:
        mSelectionSceneData->setActive(true);
        mSelectionVertices->setActive(false);
        break;
    case SELECT_VERTICES:
        mSelectionSceneData->setActive(false);
        mSelectionVertices->setActive(true);
        break;
    case UNDEFINED:
        break;
    }
    updateModels();
}

void SelectionControl::initiateNewSelection(int x, int y)
{
    switch (mSelectionMode)
    {
    case RAY:
        initiateNewSelectionRay(x, y);
        break;
    case RECTANGLE:
        initiateNewSelectionRectangle(x, y);
        break;
    }
}

void SelectionControl::updateSelection(int x, int y)
{
    switch (mSelectionMode)
    {
    case RAY:
        updateSelectionRay(x, y);
        break;
    case RECTANGLE:
        updateSelectionRectangle(x, y);
        break;
    }
}

void SelectionControl::clearSelection()
{
    switch(mSelectionType)
    {
    case SELECT_SCENE_NODES:
        mSelectionSceneData->clear();
        break;
    case SELECT_VERTICES:
        mSelectionVertices->clear();
        break;
    case UNDEFINED:
        break;
    }
}

void SelectionControl::clearSelectionMode()
{
    switch (mSelectionMode)
    {
    case RAY:
//        mSelectionRay->clear();
        break;
    case RECTANGLE:
        mSelectionRectangle->setActive(false);
        break;
    }
}

void SelectionControl::selectSceneNode(SGNode* node)
{
    std::set<std::shared_ptr<SceneData>> sceneDatas;
    VertexCollection vc;

    SGTraverser traverser = SGTraverserFactory::createDefaultSGTraverser(node);
    class Visitor : public SGNodeVisitor
    {
    public:
        Visitor(SelectionControl& _sc,
                std::set<std::shared_ptr<SceneData>>& _sd,
                VertexCollection& _vc)
            : sc(_sc)
            , sceneDatas(_sd)
            , vc(_vc)
        {
        }

        virtual void visit(SGChildrenNode* /*childrenNode*/)
        {

        }

        virtual void visit(SGLeafNode* leafNode)
        {
            std::shared_ptr<SceneLeafData> data = leafNode->getData();
            switch (sc.mSelectionType)
            {
            case SELECT_SCENE_NODES:
                sceneDatas.insert(data);
                break;
            case SELECT_VERTICES:
            {
                GeometricData* gd = data->getGeometricDataRaw();
                for (ID i = 0; i < gd->getSize(); ++i)
                {
                    vc.addVertex(data, i);
                }
                break;
            }
            case UNDEFINED:
                break;
            }

            for (auto it : sc.mSelectionListeners)
                it->onSceneNodeSelected(data);
        }

        SelectionControl& sc;
        std::set<std::shared_ptr<SceneData>>& sceneDatas;
        VertexCollection& vc;
    } visitor(*this, sceneDatas, vc);

    traverser.traverse(visitor);

    updateSelection(visitor.sceneDatas, visitor.vc);
    updateModels();
}

void SelectionControl::selectSceneNodes(const std::vector<SGNode*>& nodes)
{
    std::set<std::shared_ptr<SceneData>> nodesSet;
    for (SGNode* node : nodes)
    {
        if (node->isLeaf())
            nodesSet.insert(static_cast<SGLeafNode*>(node)->getData());
        else
            nodesSet.insert(static_cast<SGChildrenNode*>(node)->getData());
    }
    updateSelection(nodesSet, VertexCollection());
    updateModels();
}

void SelectionControl::setSceneNodeSelection(const std::vector<SGNode*>& nodes)
{
    std::set<std::shared_ptr<SceneData>> nodesSet;
    for (SGNode* node : nodes)
    {
        if (node->isLeaf())
            nodesSet.insert(static_cast<SGLeafNode*>(node)->getData());
        else
            nodesSet.insert(static_cast<SGChildrenNode*>(node)->getData());
    }
    updateSelection(nodesSet);
    updateModels();
}

void SelectionControl::setVertexSelection(VertexCollection& vc)
{
    updateSelection(vc);
    updateModels();
}

SelectionControl::SelectionType SelectionControl::getSelectionType() const
{
    return mSelectionType;
}

SelectionSceneData* SelectionControl::getSelectionSceneData()
{
    return mSelectionSceneData.get();
}

SelectionVertices* SelectionControl::getSelectionVertices()
{
    return mSelectionVertices.get();
}

const std::set<std::shared_ptr<SceneData>>& SelectionControl::getSelectedSceneData()
{
    return mSelectionSceneData->getSceneData();
}

std::vector<std::shared_ptr<SceneLeafData>> SelectionControl::retrieveSelectedSceneLeafData()
{
    std::vector<std::shared_ptr<SceneLeafData>> sceneLeafData;
    for (const std::shared_ptr<SceneData>& sd : getSelectedSceneData())
    {
        if (sd->isLeafData())
        {
            sceneLeafData.push_back(std::static_pointer_cast<SceneLeafData>(sd));
        }
    }
    return sceneLeafData;
}

void SelectionControl::initiateNewSelectionRectangle(int x, int y)
{
    mSelectionRectangle->setRectangle(x, y, x, y);
    mSelectionRectangle->setActive(true);
    mSelectionRectangleModel->update();
}

void SelectionControl::updateSelectionRectangle(int xEnd, int yEnd)
{
    mSelectionRectangle->setRectangle(
                mSelectionRectangle->getXStart(),
                mSelectionRectangle->getYStart(),
                xEnd,
                yEnd);
    mSelectionRectangleModel->update();
}

void SelectionControl::cancelSelectionRectangle()
{
    mSelectionRectangle->setActive(false);
}

void SelectionControl::initiateNewSelectionRay(int /*x*/, int /*y*/)
{

}

void SelectionControl::updateSelectionRay(int /*x*/, int /*y*/)
{

}

void SelectionControl::cancelSelectionRay()
{

}

const SelectionRectangle* SelectionControl::getSelectionRectangle() const
{
    return mSelectionRectangle.get();
}

void SelectionControl::finalizeSelection(ViewFrustum& viewFrustum)
{
    std::set<std::shared_ptr<SceneData>> sceneDatas;
    VertexCollection vc;

    // update selected vertices
    // Iterate over the whole scene graph and
    // -> If SelectionType == SELECT_VERTICES: finds out which vertices are
    //    within the given viewFrustum. Stores them in the VertexCollection.
    // -> If SelectionType == SELECT_SCENE_NODES: finds out which scene nodes
    //    are within the given viewFrustum. Stores them in the sceneDatas.
    SGTraverser traverser = mAc->getSGControl()->createSceneGraphTraverser();
    class SelectionVisitor : public SGNodeVisitorImpl
    {
    public:
        SelectionVisitor(SelectionControl& _sc,
                         ViewFrustum& _viewFrustum,
                         std::set<std::shared_ptr<SceneData>>& _sceneDatas,
                         VertexCollection& _vc)
            : sc(_sc)
            , viewFrustum(_viewFrustum)
            , sceneDatas(_sceneDatas)
            , vc(_vc)
        {
        }

        void visit(SGLeafNode* leafNode)
        {
            sc.finalizeSelection(leafNode->getData(), viewFrustum, sceneDatas, vc);
        }

        SelectionControl& sc;
        ViewFrustum& viewFrustum;

        std::set<std::shared_ptr<SceneData>>& sceneDatas;
        VertexCollection& vc;

    } visitor(*this, viewFrustum, sceneDatas, vc);
    traverser.traverse(visitor);

    updateSelection(sceneDatas, vc);

    // Disable selection rectangle
    switch (mSelectionMode)
    {
    case RAY:
//        mSelectionRay->setActive(false);
        break;
    case RECTANGLE:
        mSelectionRectangle->setActive(false);
        break;
    }

    updateModels();
}

void SelectionControl::updateModels()
{
    mSelectionVerticesModel->update();
    mSelectionSceneDataModel->update();
    mSelectionRectangleModel->update();
}

void SelectionControl::addListener(SelectionListener* listener)
{
    if (std::find(mSelectionListeners.begin(),
                  mSelectionListeners.end(), listener) ==
        mSelectionListeners.end())
    {
        mSelectionListeners.push_back(listener);
    }
}

void SelectionControl::removeListener(SelectionListener* listener)
{
    auto it = std::find(mSelectionListeners.begin(),
                        mSelectionListeners.end(), listener);
    if (it != mSelectionListeners.end())
    {
        mSelectionListeners.erase(it);
    }
}

void SelectionControl::updateSelection(
        const std::set<std::shared_ptr<SceneData> >& sceneDatas,
        const VertexCollection& vc)
{
    switch (mSelectionType)
    {
    case SELECT_SCENE_NODES:
        updateSelection(sceneDatas);
        break;
    case SELECT_VERTICES:
        updateSelection(vc);
        break;
    case UNDEFINED:
        break;
    }
}

void SelectionControl::updateSelection(const std::set<std::shared_ptr<SceneData> >& sceneDatas)
{
    mSelectionSceneData->updateSelection(sceneDatas);

    for (auto it : mSelectionListeners)
    {
        it->onSelectedSceneNodesChanged(mSelectionSceneData->getSceneData());
    }
}

void SelectionControl::updateSelection(const VertexCollection& vc)
{
    mSelectionVertices->updateSelectedVertices(vc);

    for (auto it : mSelectionListeners)
    {
        it->onSelectedVerticesChanged(
                    mSelectionVertices->getDataVectorsMap());
    }
}

void SelectionControl::finalizeSelection(
        const std::shared_ptr<SceneLeafData>& leafData,
        ViewFrustum& viewFrustum,
        std::set<std::shared_ptr<SceneData>>& sceneDatas,
        VertexCollection& vc)
{
    switch(mSelectionType)
    {
    case SELECT_SCENE_NODES:

        switch (mSelectionMode)
        {
        case RECTANGLE:
            mSelectionSceneData->calculateSelectionByRectangle(
                        leafData, &viewFrustum, *mSelectionRectangle.get(),
                        sceneDatas);
            break;
        case RAY:
            mSelectionSceneData->calculateSelectionByRay(
                        leafData, &viewFrustum,
                        mSelectionRectangle->getXEnd(),
                        mSelectionRectangle->getYEnd(),
                        sceneDatas);
            break;
        }

        break;
    case SELECT_VERTICES:
        switch (mSelectionMode)
        {
        case RECTANGLE:
            mSelectionVertices->calculateSelectionByRectangle(
                        leafData, &viewFrustum, *mSelectionRectangle.get(), vc);
            break;
        case RAY:
            mSelectionVertices->calculateSelectionByRay(
                        leafData, &viewFrustum,
                        mSelectionRectangle->getXEnd(),
                        mSelectionRectangle->getYEnd(), vc);
            break;
        }

        break;
    case UNDEFINED:
        break;
    }
}

