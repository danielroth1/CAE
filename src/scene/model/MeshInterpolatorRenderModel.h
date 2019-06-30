#ifndef MESHINTERPOLATORRENDERMODEL_H
#define MESHINTERPOLATORRENDERMODEL_H

#include "RenderModel.h"

class MeshInterpolator;
class RenderLines;
class RenderPoints;


class MeshInterpolatorRenderModel : public RenderModel
{
public:
    MeshInterpolatorRenderModel(
            std::shared_ptr<MeshInterpolator> interpolator,
            bool renderPointsEnablded = true,
            bool renderLinesEnablded = true);

    void setRenderPointsEnabled(bool renderPointsEnabled);
    bool isRenderPointsEnabled() const;

    void setRenderLinesEnabled(bool renderLinesEnabled);
    bool isRenderLinesEnabled() const;

    // RenderModel interface
public:
    virtual void reset();
    virtual void update();
    virtual void revalidate();
    virtual void accept(RenderModelVisitor& v);
    virtual void addToRenderer(Renderer* renderer);
    virtual void removeFromRenderer(Renderer* renderer);

private:

    std::shared_ptr<MeshInterpolator> mInterpolator;

    Renderer* mRenderer;

    std::shared_ptr<RenderLines> mRenderLines;
    std::shared_ptr<RenderPoints> mRenderPoints;

    bool mRenderPointsEnabled;
    bool mRenderLinesEnabled;
};

#endif // MESHINTERPOLATORRENDERMODEL_H
