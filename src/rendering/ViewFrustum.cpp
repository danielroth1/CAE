#include "ViewFrustum.h"

ViewFrustum::ViewFrustum()
{

}

const double *ViewFrustum::getModelView() const
{
    return m_modelview_matrix;
}

double *ViewFrustum::getModelView()
{
    return m_modelview_matrix;
}

const double *ViewFrustum::getProjection() const
{
    return m_projection_matrix;
}

double *ViewFrustum::getProjection()
{
    return m_projection_matrix;
}

const int *ViewFrustum::getViewPort() const
{
    return m_view_port;
}

int *ViewFrustum::getViewPort()
{
    return m_view_port;
}
