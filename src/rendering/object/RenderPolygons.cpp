#include "RenderPolygons.h"
#include "RenderPolygonsConstantDataBS.h"
#include "RenderPolygonsConstantDataWS.h"
#include "RenderPolygonsDataBS.h"
#include "RenderPolygonsDataWS.h"

#include <times/timing.h>

#include <multi_threading/Domain.h>

RenderPolygons::RenderPolygons(
        std::shared_ptr<RenderPolygonsConstantDataBS>& constantData)
    : RenderObject()
    , mType(BSWSVectors::Type::BODY_SPACE)
    , mConstantData(constantData)
{
    mDomain = nullptr;
}

RenderPolygons::RenderPolygons(
        std::shared_ptr<RenderPolygonsConstantDataWS>& constantData)
    : RenderObject()
    , mType(BSWSVectors::Type::WORLD_SPACE)
    , mConstantData(constantData)
{
    mDomain = nullptr;
}

RenderPolygons::~RenderPolygons()
{

}

void RenderPolygons::setDomain(Domain* domain)
{
    mDomain = domain;
    mProxy = std::make_shared<RenderPolygonsProxy>(this);
}

Domain* RenderPolygons::getDomain()
{
    return mDomain;
}

BSWSVectors::Type RenderPolygons::getType() const
{
    return mType;
}

void RenderPolygons::addRenderPolygonsData(
        std::shared_ptr<RenderPolygonsData> data)
{
    if (mProxy)
        mProxy->addRenderPolygonsDataSlot(data);
}

void RenderPolygons::removeRenderPolygonsData(
        std::shared_ptr<RenderPolygonsData> data)
{
    if (mProxy)
        mProxy->removeRenderPolygonsDataSlot(data);
}

void RenderPolygons::addRenderPolygonsDataSlot(
        std::shared_ptr<RenderPolygonsData> data)
{
    auto it = std::find(mData.begin(), mData.end(), data);
    if (it == mData.end())
    {
        mData.push_back(data);
    }
}

void RenderPolygons::removeRenderPolygonsDataSlot(
        std::shared_ptr<RenderPolygonsData> data)
{
    auto it = std::find(mData.begin(), mData.end(), data);
    if (it != mData.end())
    {
        mData.erase(it);
    }
}

std::vector<std::shared_ptr<RenderPolygonsData>>& RenderPolygons::getData()
{
    return mData;
}

std::shared_ptr<RenderPolygonsConstantData>& RenderPolygons::getConstantData()
{
    return mConstantData;
}

void RenderPolygons::accept(RenderObjectVisitor& /*visitor*/)
{

}

void RenderPolygons::draw()
{
    glColor3f(0.2f, 0.5f, 0.8f);
    drawVBO();
}

void RenderPolygons::drawImmediate()
{
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    glPushMatrix();

    switch(mType)
    {
    case BSWSVectors::BODY_SPACE:
    {
        RenderPolygonsConstantDataBS* constantData =
                static_cast<RenderPolygonsConstantDataBS*>(mConstantData.get());

        if (constantData->getFacesBuffer().getData().lock()->size() == 0)
            return;

        for (const std::shared_ptr<RenderPolygonsData>& data : mData)
        {
            RenderPolygonsDataBS* dataBS = static_cast<RenderPolygonsDataBS*>(data.get());

            glMultMatrixf(dataBS->getTransform()->data());
            glBegin(GL_TRIANGLES);

            auto positionsLock = constantData->getPositionsBuffer().getData().lock();
            auto normalsLock = constantData->getNormalsBuffer().getData().lock();
            auto facesLock = constantData->getFacesBuffer().getData().lock();

            for (const Face& f : *facesLock)
            {
                for (unsigned int i = 0; i < 3; ++i)
                {
                    unsigned int index = f[i];
                    const Eigen::Vector3f& v = (*positionsLock)[index];
                    const Eigen::Vector3f& n = (*normalsLock)[index];

                    glNormal3f(n(0), n(1), n(2));
                    glVertex3f(v(0), v(1), v(2));
                }
            }

            glEnd();
        }

        break;
    }
    case  BSWSVectors::WORLD_SPACE:
    {
        RenderPolygonsConstantDataWS* constantData =
                static_cast<RenderPolygonsConstantDataWS*>(mConstantData.get());

        if (constantData->getFacesBuffer().getData().lock()->size() == 0)
            return;

        for (const std::shared_ptr<RenderPolygonsData>& data : mData)
        {
            RenderPolygonsDataWS* dataWS =
                    static_cast<RenderPolygonsDataWS*>(data.get());

            glBegin(GL_TRIANGLES);

            auto positionsLock = dataWS->getPositionsBuffer().getData().lock();
            auto normalsLock = dataWS->getNormalsBuffer().getData().lock();
            auto facesLock = constantData->getFacesBuffer().getData().lock();

            for (const Face& f : *facesLock)
            {
                for (unsigned int i = 0; i < 3; ++i)
                {
                    unsigned int index = f[i];
                    const Eigen::Vector3f& v = (*positionsLock)[index];
                    const Eigen::Vector3f& n = (*normalsLock)[index];

                    glNormal3f(n(0), n(1), n(2));
                    glVertex3f(v(0), v(1), v(2));
                }
            }

            glEnd();
        }
        break;
    }
    }

    glPopMatrix();
}

void RenderPolygons::drawArray()
{


    switch(mType)
    {
    case BSWSVectors::BODY_SPACE:
    {
        RenderPolygonsConstantDataBS* constantData =
                static_cast<RenderPolygonsConstantDataBS*>(mConstantData.get());

        if (constantData->getFacesBuffer().getData().lock()->size() == 0)
            return;

        int drawCount = 0;
        for (const std::shared_ptr<RenderPolygonsData>& data : mData)
        {
            if (!data->isInitialized() || !data->isVisible())
                continue;
            ++drawCount;

            RenderPolygonsDataBS* dataBS = static_cast<RenderPolygonsDataBS*>(data.get());

            glPushMatrix();
            glMultMatrixf(dataBS->getTransform()->data());

            // draw in array mode
            glEnableClientState(GL_VERTEX_ARRAY);
            glEnableClientState(GL_NORMAL_ARRAY);

            // assign triangle data
            {
                auto positionsLock = constantData->getPositionsBuffer().getData().lock();
                glVertexPointer(3, GL_FLOAT, 0, positionsLock->data());
            }

            // assign normals
            {
                auto normalsLock = constantData->getNormalsBuffer().getData().lock();
                glNormalPointer(GL_FLOAT, 0, normalsLock->data());
            }

            // draw triangles
            {
                auto facesLock = constantData->getFacesBuffer().getData().lock();
                glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(facesLock->size()*3),
                               GL_UNSIGNED_INT, facesLock->data());
            }

            glDisableClientState(GL_VERTEX_ARRAY);
            glDisableClientState(GL_NORMAL_ARRAY);

            glPopMatrix();
        }
        std::cout << "drawCount = " << drawCount << "/" << mData.size() << "\n";

        break;
    }
    case  BSWSVectors::WORLD_SPACE:
    {
        RenderPolygonsConstantDataWS* constantData =
                static_cast<RenderPolygonsConstantDataWS*>(mConstantData.get());

        if (constantData->getFacesBuffer().getData().lock()->size() == 0)
            return;

        for (const std::shared_ptr<RenderPolygonsData>& data : mData)
        {
            if (!data->isInitialized() || !data->isVisible())
                continue;

            RenderPolygonsDataWS* dataBS =
                    static_cast<RenderPolygonsDataWS*>(data.get());

            // draw in array mode
            glEnableClientState(GL_VERTEX_ARRAY);
            glEnableClientState(GL_NORMAL_ARRAY);

            // assign triangle data
            {
                auto positionsLock = dataBS->getPositionsBuffer().getData().lock();
                glVertexPointer(3, GL_FLOAT, 0, positionsLock->data());
            }

            // assign normals
            {
                auto normalsLock = dataBS->getNormalsBuffer().getData().lock();
                glNormalPointer(GL_FLOAT, 0, normalsLock->data());
            }

            // draw triangles
            {
                auto facesLock = constantData->getFacesBuffer().getData().lock();
                glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(facesLock->size()*3),
                               GL_UNSIGNED_INT, facesLock->data());
            }

            glDisableClientState(GL_VERTEX_ARRAY);
            glDisableClientState(GL_NORMAL_ARRAY);
        }


        break;
    }
    }


}

void RenderPolygons::drawVBO()
{
    START_TIMING_RENDERING("RenderPolygons::drawVBO2");
    if (!mVBOsupported || !mConstantData->isInitialized())
        return;

    switch(mType)
    {
    case BSWSVectors::BODY_SPACE:
    {
        RenderPolygonsConstantDataBS* constantData =
                static_cast<RenderPolygonsConstantDataBS*>(mConstantData.get());

        if (!constantData->isInitialized())
            constantData->initialize();

//        if (!constantData->getFacesBuffer().isInitialized() ||
//            !constantData->getNormalsBuffer().isInitialized() ||
//            !constantData->getPositionsBuffer().isInitialized())
//            return;

        glEnableClientState(GL_NORMAL_ARRAY);
        glEnableClientState(GL_VERTEX_ARRAY);
        constantData->getNormalsBuffer().bindBuffer();
        glNormalPointer(GL_FLOAT, 0, nullptr);
        constantData->getPositionsBuffer().bindBuffer();
        glVertexPointer(3, GL_FLOAT, 0, nullptr);
        constantData->getFacesBuffer().bindBuffer();

        int nDrawnElements = 0;
        for (const std::shared_ptr<RenderPolygonsData>& data : mData)
        {
            if (!data->isVisible())
                continue;

            if (!data->isInitialized())
                data->initialize();

            glColor4fv(data.get()->getColor().data());
            // TODO: the constant datas buffers are created but
            // are they also filled with the correct data? call
            // refresh or sth.
            RenderPolygonsDataBS* dataBS =
                    static_cast<RenderPolygonsDataBS*>(data.get());

            glPushMatrix();
            glMultMatrixf(dataBS->getTransform()->data());
            glDrawElements(
                        GL_TRIANGLES,
                        static_cast<int>(
                            3 * constantData->getFacesBuffer().getData().unsafe().size()),
                        GL_UNSIGNED_INT,
                        nullptr);
            glPopMatrix();
            ++nDrawnElements;
        }
//        std::cout << "BS: nDrawnElements = " << nDrawnElements << "/" << mData.size() << "\n";

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        break;
    }
    case  BSWSVectors::WORLD_SPACE:
    {
        RenderPolygonsConstantDataWS* constantData =
                static_cast<RenderPolygonsConstantDataWS*>(mConstantData.get());

        if (!constantData->isInitialized())
            constantData->initialize();

        constantData->getFacesBuffer().bindBuffer();

        for (const std::shared_ptr<RenderPolygonsData>& data : mData)
        {
            if (!data->isVisible())
                continue;

            if (!data->isInitialized())
                data->initialize();

            glColor4fv(data.get()->getColor().data());

            RenderPolygonsDataWS* dataWS = static_cast<RenderPolygonsDataWS*>(data.get());

            glEnableClientState(GL_NORMAL_ARRAY);
            glEnableClientState(GL_VERTEX_ARRAY);
            dataWS->getNormalsBuffer().bindBuffer();
            glNormalPointer(GL_FLOAT, 0, nullptr);
            dataWS->getPositionsBuffer().bindBuffer();
            glVertexPointer(3, GL_FLOAT, 0, nullptr);

            glDrawElements(
                        GL_TRIANGLES,
                        static_cast<int>(3 * constantData->getFacesBuffer().getData().unsafe().size()),
                        GL_UNSIGNED_INT,
                        nullptr);

            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glDisableClientState(GL_VERTEX_ARRAY);
            glDisableClientState(GL_NORMAL_ARRAY);

        }

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        break;
    }
    }

    STOP_TIMING_RENDERING;
}

void RenderPolygons::update()
{

}

void RenderPolygons::createBuffers()
{
    // TODO: createBuffers() is never called because it is only called
    // once for RenderPolygons (before all the data is added)
    if (glGenBuffers && glBindBuffer && glBufferData && glBufferSubData
            && glMapBuffer && glUnmapBuffer && glDeleteBuffers
            && glGetBufferParameteriv)
        mVBOsupported = true;
    else
    {
        mVBOsupported = false;
        std::cout << "VBOs are not supported!";
    }

    for (std::shared_ptr<RenderPolygonsData>& data : mData)
    {
        data->createBuffers();
    }
    mConstantData->createBuffers();
}

void RenderPolygons::refreshBuffers()
{
    for (std::shared_ptr<RenderPolygonsData>& data : mData)
    {
        data->refreshBuffers();
    }
    mConstantData->refreshBuffers();
}

void RenderPolygons::cleanup()
{
    for (std::shared_ptr<RenderPolygonsData>& data : mData)
    {
        data->cleanup();
    }
    mConstantData->cleanup();
}

void RenderPolygons::initialize()
{
    for (std::shared_ptr<RenderPolygonsData>& data : mData)
    {
        data->initialize();
    }
    mConstantData->initialize();
}
