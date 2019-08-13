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
#include <scene/VertexCollection.h>
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
    switch (mSelectionType)
    {
    case SELECT_SCENE_NODES:
        mSelectionSceneData->getSceneData().clear();
        break;
    case SELECT_VERTICES:
    {
        mSelectionVertices->clear();
        break;
    }
    case UNDEFINED:
        break;
    }

    SGTraverser traverser = SGTraverserFactory::createDefaultSGTraverser(node);
    class Visitor : public SGNodeVisitor
    {
    public:
        Visitor(SelectionControl& _sc)
            : sc(_sc)
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
                sc.mSelectionSceneData->getSceneData().insert(data);
                break;
            case SELECT_VERTICES:
            {
                GeometricData* gd = data->getGeometricDataRaw();
                for (ID i = 0; i < gd->getSize(); ++i)
                {
                    sc.mSelectionVertices->getSelectedVertexCollection()
                            ->addVertex(data, i);
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
    } visitor(*this);

    traverser.traverse(visitor);

    updateModels();
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
    // if no shift is pressed
    clearSelection();

    // update selected vertices
    // TODO: simpler way of traversing scene leaf nodes?
    SGTraverser traverser = mAc->getSGControl()->createSceneGraphTraverser();
    class SelectionVisitor : public SGNodeVisitorImpl
    {
    public:
        SelectionVisitor(SelectionControl& _sc, ViewFrustum& _viewFrustum)
            : sc(_sc)
            , viewFrustum(_viewFrustum)
        {
        }

        void visit(SGLeafNode* leafNode)
        {
            sc.finalizeSelection(leafNode->getData(), viewFrustum);
        }

        SelectionControl& sc;
        ViewFrustum& viewFrustum;
    } visitor(*this, viewFrustum);
    traverser.traverse(visitor);

    switch(mSelectionType)
    {
    case SELECT_SCENE_NODES:
    {
        for (auto it : mSelectionListeners)
        {
            it->onSelectedSceneNodesChanged(mSelectionSceneData->getSceneData());
        }
        break;
    }
    case SELECT_VERTICES:
    {
        for (auto it : mSelectionListeners)
        {
            it->onSelectedVerticesChanged(
                        mSelectionVertices->getSelectedVertexCollection()->getDataVectorsMap());
        }
        break;
    }
    case UNDEFINED:
        break;
    }


    std::cout << mSelectionVertices->getSelectedVertexCollection()->getDataVectorsMap().size() << std::endl;

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

void SelectionControl::finalizeSelection(
        const std::shared_ptr<SceneLeafData>& leafData,
        ViewFrustum& viewFrustum)
{
    switch(mSelectionType)
    {
    case SELECT_SCENE_NODES:

        switch (mSelectionMode)
        {
        case RECTANGLE:
            mSelectionSceneData->updateSelectionByRectangle(
                        leafData, &viewFrustum, *mSelectionRectangle.get());
            break;
        case RAY:
            mSelectionSceneData->updateSelectionByRay(
                        leafData, &viewFrustum,
                        mSelectionRectangle->getXEnd(),
                        mSelectionRectangle->getYEnd());
            break;
        }

        break;
    case SELECT_VERTICES:
        switch (mSelectionMode)
        {
        case RECTANGLE:
            mSelectionVertices->updateSelectionByRectangle(
                        leafData, &viewFrustum, *mSelectionRectangle.get());
            break;
        case RAY:
            mSelectionVertices->updateSelectionByRay(
                        leafData, &viewFrustum,
                        mSelectionRectangle->getXEnd(),
                        mSelectionRectangle->getYEnd());
            break;
        }

        break;
    case UNDEFINED:
        break;
    }
}

