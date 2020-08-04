#ifndef MESHINTERPOLATORRENDERMODEL_H
#define MESHINTERPOLATORRENDERMODEL_H

#include "RenderModel.h"

class MeshInterpolator;
class RenderLines;
class RenderPoints;


// Draws lines for each target vertex from the target vertex to the
// corresponding source vertex.
//
// Doesn't work for the MeshInterpolatorFEM because it maps from
// tetrahedrons to target positions and there is no easy way to visualize this.
// One could draw lines from each vertex of the corresponding tetrahedron to
// the target vertex but this would quickly become confusing.
class MeshInterpolatorRenderModel : public RenderModel
{
public:
    MeshInterpolatorRenderModel(
            std::shared_ptr<MeshInterpolator> interpolator,
            bool renderPointsEnablded = false,
            bool renderLinesEnablded = true);

    virtual ~MeshInterpolatorRenderModel() override;

    void setRenderPointsEnabled(bool renderPointsEnabled);
    bool isRenderPointsEnabled() const;

    void setRenderLinesEnabled(bool renderLinesEnabled);
    bool isRenderLinesEnabled() const;

    // RenderModel interface
public:
    virtual void reset() override;
    virtual void update() override;
    virtual void revalidate() override;
    virtual void accept(RenderModelVisitor& v) override;
    virtual void addToRenderer(Renderer* renderer) override;
    virtual void removeFromRenderer(Renderer* renderer) override;
    virtual void setVisible(bool visible) override;

private:

    std::shared_ptr<MeshInterpolator> mInterpolator;

    Renderer* mRenderer;

    std::shared_ptr<RenderLines> mRenderLines;
    std::shared_ptr<RenderPoints> mRenderPoints;

    bool mRenderPointsEnabled;
    bool mRenderLinesEnabled;
};

#endif // MESHINTERPOLATORRENDERMODEL_H
