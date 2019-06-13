#ifndef APPEARANCES_H
#define APPEARANCES_H


#include <memory>
#include <vector>

class Appearance;

class Appearances
{
public:
    // Sets the standard appearance to white.
    Appearances();

    // Constructor with standard appearance.
    Appearances(std::shared_ptr<Appearance> standardAppearance);

    // Adds an appearance.
    // The appearance affects all triangles with the ids starting
    // from the one from the previous added triangle to the
    // ones plus the number of triangles.
    // \param triangleIds - the number of triangles that are
    //      affected by the appearance
    void addAppearance(std::shared_ptr<Appearance> appearance,
                       unsigned int triangleIds);


    std::shared_ptr<Appearance> getStandardAppearances() const;

    // Returns number of offsets and appearances.
    std::size_t getSize() const;

    unsigned int getOffset(std::size_t index) const;
    std::size_t getOffsetsSize() const;

    std::shared_ptr<Appearance> getAppearance(std::size_t index) const;

private:
    std::shared_ptr<Appearance> mStandardAppearance;

    std::vector<unsigned int> mOffsets;
    std::vector<std::shared_ptr<Appearance>> mAppearances;

};

#endif // APPEARANCES_H
