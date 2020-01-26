#ifndef TETGENIMPORTER_H
#define TETGENIMPORTER_H

#include "SceneImporter.h"


// Importer for (.node, .face) or (.node, .ele) file pairs.
class TetGenImporter : public SceneImporter
{
public:
    TetGenImporter();

    virtual SGNode* importNodeFace(
            const File& nodeFile,
            const File& faceFile,
            ApplicationControl* ac);

    virtual SGNode* importNodeEle(
            const File& nodeFile,
            const File& eleFile,
            ApplicationControl* ac);

    // Automatically checks which file combination (either (.node, .face)
    // or (.node, .ele)) is in the given files and imports them.
    // If both are given, (.node, .face) will be preferred.
    virtual SGNode* importFiles(
            const std::vector<File>& files,
            ApplicationControl* ac);

    // SceneImporter interface
public:
    virtual std::string getFileFormat() const;
    // Unused
    virtual SGNode* importFile(File file, ApplicationControl* ac);

private:
    Vectors importNode(const File& nodeFile);
    Faces importFace(const File& faceFile);
    Cells importEle(const File& eleFile);

};

#endif // TETGENIMPORTER_H
