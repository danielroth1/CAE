#include "ElasticMaterial.h"

ElasticMaterial::ElasticMaterial()
{
    mYoungsModulus = 0.0;
    mPoissonRatio = 0.0;
    mLameMu = 0.0;
    mLameLambda = 0.0;

    mPlasticYield = 0.0;
    mPlasticCreep = 0.0;
    mPlasticMaxStrain = 0.0;
}

void ElasticMaterial::setFromLame(double lameMu, double lameLambda)
{
    mLameMu = lameMu;
    mLameLambda = lameLambda;
    mYoungsModulus = lameMu * (3 * lameLambda + 2 * lameMu) /
            (lameLambda + lameMu);
    mPoissonRatio = lameLambda / (2 * (lameLambda + lameMu));
}

void ElasticMaterial::setFromYoungsPoisson(
        double youngsModulus,
        double poissonRatio)
{
    mYoungsModulus = youngsModulus;
    mPoissonRatio = poissonRatio;
    mLameMu = youngsModulus /
            (2 * (1 + poissonRatio));
    mLameLambda = youngsModulus * poissonRatio /
            ( ( 1 + poissonRatio ) * ( 1 - 2 * poissonRatio ) );
}

void ElasticMaterial::setPlasticYield(double plasticYield)
{
    mPlasticYield = plasticYield;
}

void ElasticMaterial::setPlasticCreep(double plasticCreep)
{
    mPlasticCreep = plasticCreep;
}

void ElasticMaterial::setPlasticMaxStrain(double plasticMaxStrain)
{
    mPlasticMaxStrain = plasticMaxStrain;
}

double ElasticMaterial::getYoungsModulus() const
{
    return mYoungsModulus;
}

double ElasticMaterial::getPoissonRatio() const
{
    return mPoissonRatio;
}

double ElasticMaterial::getLameLambda() const
{
    return mLameLambda;
}

double ElasticMaterial::getLameMu() const
{
    return mLameMu;
}

double ElasticMaterial::getPlasticYield() const
{
    return mPlasticYield;
}

double ElasticMaterial::getPlasticCreep() const
{
    return mPlasticCreep;
}

double ElasticMaterial::getPlasticMaxStrain() const
{
    return mPlasticMaxStrain;
}
