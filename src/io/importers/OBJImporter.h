#ifndef OBJIMPORTER_H
#define OBJIMPORTER_H

#include "SceneImporter.h"


class Appearance;
class ApplicationControl;
class RenderMaterial;

class OBJImporter : public SceneImporter
{
public:
    OBJImporter();

    // SceneImporter interface
public:
    virtual std::string getFileFormat() const;
    virtual SGNode* importFile(File file, ApplicationControl* ac);

private:
    std::vector<std::shared_ptr<Appearance>> readMtllib(File file);

    void createAndAddAppearance(
            std::vector<std::shared_ptr<Appearance>>& appearances,
            const std::shared_ptr<RenderMaterial>& renderMaterial,
            const File& directory,
            const std::string& name,
            const std::string& textureName,
            float opaqueness);
};

#endif // OBJIMPORTER_H
