#ifndef VIEWFRUSTUM_H
#define VIEWFRUSTUM_H


class ViewFrustum
{
public:
    ViewFrustum();

    // return the model view matrix
    const double* getModelView() const;
    double* getModelView();

    // return the projection matrix
    const double* getProjection() const;
    double* getProjection();

    // return the view port array
    // VP[0] = x_min = 0
    // VP[1] = y_min = 0
    // VP[2] = x_max = height
    // VP[3] = y_max = width
    const int* getViewPort() const;
    int* getViewPort();

private:
    double m_modelview_matrix[16];
    double m_projection_matrix[16];
    int m_view_port[4];
};

#endif // VIEWFRUSTUM_H
