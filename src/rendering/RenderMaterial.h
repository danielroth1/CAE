#ifndef RENDERMATERIAL_H
#define RENDERMATERIAL_H

#include <array>
#include <memory>


// This class describes a material used for rendering. It has a
// ambient, diffuse, specular, and shinines component. Use glMaterial()
// to call the corresponding opengl method glMaterial for all specified
// materials.
class RenderMaterial
{
public:

    // Initializes the default values according to the documentation for
    // glMaterial:
    // Ambient = {0.2f, 0.2f, 0.2f, 1.0f}
    // Diffuse = {0.8f, 0.8f, 0.8f, 1.0f}
    // Specular = {0.0f, 0.0f, 0.0f, 1.0f}
    // Shininess = 0.0f
    RenderMaterial(
            const std::array<float, 4>& ambient = getAmbientDefault(),
            const std::array<float, 4>& diffuse = getDiffuseDefault(),
            const std::array<float, 4>& specular = getSpecularDefault(),
            float shininess = getShininessDefault());

    void glMaterial();

    void setOpaquness(float opaqueness);

    std::array<float, 4> getAmbient() const;
    void setAmbient(const std::array<float, 4>& ambient);

    std::array<float, 4> getDiffuse() const;
    void setDiffuse(const std::array<float, 4>& diffuse);

    std::array<float, 4> getSpecular() const;
    void setSpecular(const std::array<float, 4>& specular);

    float getShininess() const;
    void setShininess(float shininess);

    static std::array<float, 4> getAmbientDefault();
    static std::array<float, 4> getDiffuseDefault();
    static std::array<float, 4> getSpecularDefault();
    static float getShininessDefault();

    // Creates a material with the specified color.
    // Sets the ambient and diffuse part to the given color and uses for
    // the others the default values.
    static std::shared_ptr<RenderMaterial> createFromColor(
            const std::array<float, 4>& color);

private:
    std::array<float, 4> mAmbient;
    std::array<float, 4> mDiffuse;
    std::array<float, 4> mSpecular;
    float mShininess;

    bool mAmbientAndDiffuseIdentical;

};

#endif // RENDERMATERIAL_H
