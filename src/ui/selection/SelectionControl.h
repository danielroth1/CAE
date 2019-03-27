#ifndef SELECTIONCONTROL_H
#define SELECTIONCONTROL_H

#include <scene/scene_graph/SGCore.h>
#include <memory>
#include <set>
#include <vector>

class ApplicationControl;
class Renderer;
class SceneLeafData;
class SelectionListener;
class SelectionRectangle;
class SelectionRectangleModel;
class SelectionSceneData;
class SelectionSceneDataModel;
class SelectionVertices;
class SelectionVerticesModel;
class ViewFrustum;

// Controls Selection and SelectionModel.
// Is owner of both.
// Selection methods: Selection Rectangle and Selection Ray
// Selection modes: Select vertices and select scene nodes
// Shift, Ctrl
// A selection is:
//   1. initialized: draws a selection rectangle
//   2. updates: marks every object that would be selected orange (this color?)
//   3. finalized: adds the to be selected objects to the vector of selected objects
class SelectionControl
{
public:
    enum SelectionType
    {
        SELECT_VERTICES, SELECT_SCENE_NODES, UNDEFINED
    };

    // TODO: switch type and mode
    // The type specifies with which
    enum SelectionMode
    {
        RECTANGLE, RAY
    };

    SelectionControl(
            ApplicationControl* ac,
            ViewFrustum& viewFrustum);

    //*********************************************
    // General methods
        // Adds SelectionModel to renderer
        void init(Renderer* renderer);

        // Changes the selection type. Deactivates the
        // current type and activates the new one. If
        // the given type and the current one are identical
        // nothing happens.
        void changeSelectionType(SelectionType type);

        void initiateNewSelection(int x, int y);

        void updateSelection(int x, int y);

        // Updates the selection of vertices or scene data
        // depending on the chosen SelectionMode
        void finalizeSelection(ViewFrustum& viewFrustum);

        // Clears selection of vertices/ triangles as well
        // as the selection rectangle or ray.
        void clearSelection();
        void clearSelectionMode();

        // Call when a scene node is selected in the scene graph.
        // If SelectionType == SELECT_VERTICES:
        //      selects all vertices of the scene leaf data
        // If SelectionType == SELECT_SCENE_NODES
        //      selects the scene leaf data
        // At the end, updates all models.
        //\param sceneLeafData of the scene node
        void selectSceneNode(SGNode* sceneLeafData);

    //*********************************************

    // Getters
        SelectionSceneData* getSelectionSceneData();
        SelectionVertices* getSelectionVertices();
        const std::set<SceneData*>& getSelectedSceneData();
        std::vector<SceneLeafData*> retrieveSelectedSceneLeafData();

    //*********************************************
    // Selection Rectangle
    // Update Methods

        const SelectionRectangle* getSelectionRectangle() const;
        void updateModels();
    //*********************************************

    // Listener Support
        void addListener(SelectionListener* listener);
        void removeListener(SelectionListener* listener);

private:

    // General Methods
        void finalizeSelection(
                SceneLeafData* leafData,
                ViewFrustum& viewFrustum);

    // SelectionRectangle
        // Initiates a new seleciton rectangle at the given position.
        // There is only a dot visible because the rectangle has
        // initially size zero.
        void initiateNewSelectionRectangle(
                int x, int y);

        // Updates the current selection rectangle. Call
        // initiateNewSelectionRectangle() before calling this method.
        // Starting points are also defined in initiateNewSelectionRectangle().
        // Updates xEnd and yEnd only. Lets xStart, yStart
        // as is.
        void updateSelectionRectangle(
                int xEnd,
                int yEnd);

        // Finalizes the selection with a selection rectangle. Depending on
        // the chosen selection mode, either vertices or triangles that are
        // within or intersect with the selection rectangle area are selected
        // and added to the selection list.
        void finalizeSelectionRectangle(
                ViewFrustum& viewFrustum);

        // Cancels the currently active selection rectangle. Call this
        // method after calling initiateNewSelectionRectangle(). If it
        // was not called, nothing happends.
        void cancelSelectionRectangle();

    // SelectionRay
        void initiateNewSelectionRay(
                int x, int y);

        void updateSelectionRay(
                int x, int y);

        void cancelSelectionRay();
        // impelement both version, distinguish between modes

    ApplicationControl* mAc;
    SelectionMode mSelectionMode;
    SelectionType mSelectionType;

    // Listener Support
        std::vector<SelectionListener*> mSelectionListeners;

    // The rectangle that the use can draw to select multiple
    // objects in the scene at once.
        std::unique_ptr<SelectionRectangle> mSelectionRectangle;
        std::unique_ptr<SelectionRectangleModel> mSelectionRectangleModel;

    // Selection Vertices
    // Handles the selection of single vertices
        std::unique_ptr<SelectionVertices> mSelectionVertices;
        std::unique_ptr<SelectionVerticesModel> mSelectionVerticesModel;

    // Selection Scene Data
    // Handles the selection of objects in the scene
        std::unique_ptr<SelectionSceneData> mSelectionSceneData;
        std::unique_ptr<SelectionSceneDataModel> mSelectionSceneDataModel;
};

#endif // SELECTIONCONTROL_H
