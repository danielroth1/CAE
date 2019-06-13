#ifndef SCENEIMPORTER_H
#define SCENEIMPORTER_H

#include <scene/scene_graph/SGCore.h>

#include <io/File.h>

class ApplicationControl;

class SceneImporter
{
public:

    virtual std::string getFileFormat() const = 0;

    virtual SGNode* importFile(File file, ApplicationControl* ac) = 0;

protected:
    SceneImporter();
    virtual ~SceneImporter();

};

#endif // SCENEIMPORTER_H
