#ifndef ELASTICMATERIALPROPERTY_H
#define ELASTICMATERIALPROPERTY_H


// Represents elastic material parameters either by the pair of
// (Youngs Modulus, Poisson Ratio) or
// the lame parameters (Lame Lambda, Lame Mu).
// See the table at the bottom of https://en.wikipedia.org/wiki/Young%27s_modulus
// for conversion formulas.
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
