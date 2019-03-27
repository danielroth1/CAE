#ifndef ELASTICMATERIALPROPERTY_H
#define ELASTICMATERIALPROPERTY_H


class ElasticMaterial
{
public:
    ElasticMaterial();

    void setFromLame(double lameMu, double lameLambda);
    void setFromYoungsPoisson(double youngsModulus, double poissonRatio);

    double getYoungsModulus() const;
    double getPoissonRatio() const;
    double getLameLambda() const;
    double getLameMu() const;

private:
    double mYoungsModulus;
    double mPoissonRatio;

    double mLameMu;
    double mLameLambda;
};

#endif // ELASTICMATERIALPROPERTY_H
