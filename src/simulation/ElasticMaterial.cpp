#include "ElasticMaterial.h"

ElasticMaterial::ElasticMaterial()
{
    mYoungsModulus = 0.0;
    mPoissonRatio = 0.0;
    mLameMu = 0.0;
    mLameLambda = 0.0;
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
