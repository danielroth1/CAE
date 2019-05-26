#ifndef MESHCRITERIA_H
#define MESHCRITERIA_H


class MeshCriteria
{
public:
    // \param facetAngle - Pass 0 to ignore.
    // \param faceSize - Pass 0 to ignore.
    // \param facetDistance - Pass 0 to ignore.
    // \param cellSize - Pass 0 to ignore.
    // \param cellRadiusEdgeRatio - Pass 0 to ignore.
    // \param edgesAsFeatures -
    //      If edges should be used to generate features. All featured edges will
    //      be preserve in the resulting mesh. Enabling this can result in a significant
    //      higher amount of tetrahedrons. Use this when the resolution of the original
    //      mesh is low and edges need to be preserved.
    // \param minFeatureEdgeAngleDeg - [0 - 180]
    //      All edge will be features whomes angle
    //      between the edges adjacent faces
    //      is lower than this value.
    MeshCriteria(
            double facetAngle,
            double facetSize,
            double facetDistance,
            double cellSize,
            double cellRadiusEdgeRatio,
            bool edgesAsFeatures = true,
            double minFeatureEdgeAngleDeg = 60.0);

    double getFacetAngle() const;
    void setFacetAngle(double facetAngle);

    double getFacetSize() const;
    void setFacetSize(double facetSize);

    double getFaceDistance() const;
    void setFaceDistance(double faceDistance);

    double getCellSize() const;
    void setCellSize(double cellSize);

    double getCellRadiusEdgeRatio() const;
    void setCellRadiusEdgeRatio(double cellRadiusEdgeRatio);

    bool isEdgesAsFeatures() const;
    void setEdgesAsFeatures(bool edgesAsFeatures);

    double getMinFeatureEdgeAngleDeg() const;
    void setMinFeatureEdgeAngleDeg(double minFeatureEdgeAngleDeg);

private:
    double mFacetAngle;
    double mFacetSize;
    double mFaceDistance;
    double mCellSize;
    double mCellRadiusEdgeRatio;
    bool mEdgesAsFeatures;
    double mMinFeatureEdgeAngleDeg;
};

#endif // MESHCRITERIA_H
