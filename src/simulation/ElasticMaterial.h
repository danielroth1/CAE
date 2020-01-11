#ifndef ELASTICMATERIALPROPERTY_H
#define ELASTICMATERIALPROPERTY_H


// Represents elastic material parameters either by the pair of
// (Youngs Modulus, Poisson Ratio) or
// the lame parameters (Lame Lambda, Lame Mu).
// See the table at the bottom of https://en.wikipedia.org/wiki/Young%27s_modulus
// for conversion formulas.
//
// Plasticity: defined by yield, creep, and max strain
// - to disable it, just set the max strain to zero.
class ElasticMaterial
{
public:
    ElasticMaterial();

    void setFromLame(double lameMu, double lameLambda);
    void setFromYoungsPoisson(double youngsModulus, double poissonRatio);

    void setPlasticYield(double plasticYield);
    void setPlasticCreep(double plasticCreep);

    // If zero, there will be no plasticity and no plasticity related
    // calculation overhead.
    void setPlasticMaxStrain(double plasticMaxStrain);

    double getYoungsModulus() const;
    double getPoissonRatio() const;
    double getLameLambda() const;
    double getLameMu() const;

    double getPlasticYield() const;
    double getPlasticCreep() const;
    double getPlasticMaxStrain() const;

private:
    double mYoungsModulus;
    double mPoissonRatio;

    double mLameMu;
    double mLameLambda;

    double mPlasticYield;
    double mPlasticCreep;
    double mPlasticMaxStrain;
};

#endif // ELASTICMATERIALPROPERTY_H
