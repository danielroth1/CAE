#include "Appearance.h"
#include "Appearances.h"
#include "RenderMaterial.h"

Appearances::Appearances()
{
    mStandardAppearance =
            Appearance::createAppearanceFromColor({1.0f, 1.0f, 1.0f, 1.0f});
}

Appearances::Appearances(std::shared_ptr<Appearance> appearance)
{
    mStandardAppearance = appearance;
}

void Appearances::addAppearance(
        std::shared_ptr<Appearance> appearance,
        unsigned int triangleIds)
{
    mAppearances.push_back(appearance);
    mOffsets.push_back(triangleIds);
//    if (mOffsets.empty())
//        mOffsets.push_back(triangleIds);
//    else
    //        mOffsets.push_back(mOffsets[mOffsets.size()-1] + triangleIds);
}

std::shared_ptr<Appearance> Appearances::getStandardAppearances() const
{
    return mStandardAppearance;
}

std::size_t Appearances::getSize() const
{
    return mOffsets.size();
}

unsigned int Appearances::getOffset(std::size_t index) const
{
    return mOffsets[index];
}

std::size_t Appearances::getOffsetsSize() const
{
    return mOffsets.size();
}

std::shared_ptr<Appearance> Appearances::getAppearance(std::size_t index) const
{
    return mAppearances[index];
}
