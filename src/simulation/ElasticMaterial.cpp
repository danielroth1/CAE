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
    // TODO: implement this
}

void ElasticMaterial::setFromYoungsPoisson(
        double youngsModulus,
        double poissonRatio)
{

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
