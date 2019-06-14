#include "OBJImporter.h"

#include <cmath>

#include <fstream>
#include <iomanip>
#include <iostream>

#include <io/IOUtils.h>
#include <io/ImageLoader.h>

#include <map>

#include <regex>

#include <rendering/Appearance.h>
#include <rendering/Appearances.h>
#include <rendering/RenderMaterial.h>
#include <rendering/Texture.h>

#include <ApplicationControl.h>
#include <RenderModelManager.h>
#include <set>

#include <data_structures/VectorIndexMapping.h>

#include <scene/data/geometric/Polygon2D.h>

#include <scene/model/ModelUtils.h>
#include <scene/model/PolygonIndexMapping.h>
#include <scene/model/PolygonRenderModel.h>

OBJImporter::OBJImporter()
{

}

std::string OBJImporter::getFileFormat() const
{
    return "obj";
}

std::vector<std::shared_ptr<Appearance> > OBJImporter::readMtllib(File file)
{
    File directory = file;
    directory.goUpDirectory();
    std::vector<std::shared_ptr<Appearance> > appearances;

    bool successful;
    std::ifstream in = file.read(successful);
    if (!successful)
        return appearances;

    float opaqueness = 1.0f; // standard = fully opaque
    std::shared_ptr<RenderMaterial> renderMaterial = std::make_shared<RenderMaterial>();
    std::string name = "";
    std::string textureName = "";

    std::string line;
    while (std::getline(in, line))
    {
        // read line
        std::stringstream ss(line);
        std::string word;

        ss >> word;

        if (word == "newmtl")
        {
            if (name != "")
            {
                createAndAddAppearance(
                            appearances, renderMaterial, directory,
                            name, textureName, opaqueness);
            }

            // set standard values
            opaqueness = 1.0f; // standard = fully opaque
            renderMaterial = std::make_shared<RenderMaterial>();

            ss >> name;
        }
        else if (word == "Ka")
        {
            // ambient
            std::array<float, 3> color = IOUtils::readArray<float, 3>(ss);
            std::array<float, 4> color4;
            for (size_t i = 0; i < 3; ++i)
                color4[i] = color[i];
            renderMaterial->setAmbient(color4);
        }
        else if (word == "Kd")
        {
            // diffuse
            std::array<float, 3> color = IOUtils::readArray<float, 3>(ss);
            std::array<float, 4> color4;
            for (size_t i = 0; i < 3; ++i)
                color4[i] = color[i];
            renderMaterial->setDiffuse(color4);
        }
        else if (word == "Ks")
        {
            // specular
            std::array<float, 3> color = IOUtils::readArray<float, 3>(ss);
            std::array<float, 4> color4;
            for (size_t i = 0; i < 3; ++i)
                color4[i] = color[i];
            renderMaterial->setSpecular(color4);
        }
        else if (word == "Ns")
        {
            // shininess
            float shininess;
            ss >> shininess;
            renderMaterial->setShininess(shininess);
        }
        else if (word == "d")
        {
            // opaqueness
            ss >> opaqueness;
        }
        else if (word == "Tr")
        {
            // inverted opaqueness
            ss >> opaqueness;
            opaqueness = 1 - opaqueness;
        }
        else if (word == "map_Ka")
        {
            ss >> textureName;
        }
        else if (word == "map_Kd")
        {
            ss >> textureName;
        }
        else if (word == "map_Ks")
        {
            // not supported
        }
        else if (word == "map_Ns")
        {
            // not supported
        }
        else if (word == "map_d")
        {
            // not supported
        }
        else if (word == "bump")
        {
            // not supported
        }
        else if (word == "map_bump")
        {
            // not supported
        }
    }

    if (name != "")
    {
        createAndAddAppearance(
                    appearances, renderMaterial, directory,
                    name, textureName, opaqueness);
    }

    return appearances;
}

void OBJImporter::createAndAddAppearance(
        std::vector<std::shared_ptr<Appearance> >& appearances,
        const std::shared_ptr<RenderMaterial>& renderMaterial,
        const File& directory,
        const std::string& name,
        const std::string& textureName,
        float opaqueness)
{
    std::shared_ptr<Texture> texture;
    if (textureName != "")
    {
        std::shared_ptr<Image> image =
                ImageLoader::instance()->loadBMP(
                    (directory.getPath() + File::SEPARATOR + textureName).c_str());
        texture = std::make_shared<Texture>(image);
    }

    renderMaterial->setOpaquness(opaqueness);

    std::shared_ptr<Appearance> appearance;
    if (texture && renderMaterial)
        appearance = std::make_shared<Appearance>(renderMaterial, texture);
    else if (renderMaterial)
        appearance = std::make_shared<Appearance>(renderMaterial);
    appearance->setName(name);

    appearances.push_back(appearance);
}

SGNode* OBJImporter::importFile(File file, ApplicationControl* ac)
{
    bool successful;
    std::ifstream in = file.read(successful);
    if (!successful)
        return nullptr;

    std::string nodeName = "";

    Vectors verticesOriginal; // non-duplicated
    std::vector<Eigen::Vector2f> vertexTexturesOriginal;
    Vectors vertexNormalsOriginal; // non-duplicated
    Faces facesOriginal;
    Faces facesExtended;
    // original face, vertex textures, material name
    std::vector<std::tuple<Face, Face, std::string>> faceTuples;

    std::set<std::tuple<unsigned int, unsigned int, std::string>> vertexSet;

    std::string currentMaterial = "";
    std::vector<std::string> renderMaterialNames;
    std::vector<unsigned int> nFacesPerMaterial;

    std::string mtllib = "";

    std::string line;
    while (std::getline(in, line))
    {
//        line = std::regex_replace(line, std::regex("\\t"), " ");
//        std::cout << line << "\n";

        // read line
        std::stringstream ss(line);
        std::string word;

        ss >> word;

        if (word == "#")
        {
            // skip comments
            continue;
        }
        else if (word == "o")
        {
            // object name
            // Groups aren't supported at the moment
//            std::getline(ss, word);
//            nodeName = word;
        }
        else if (word == "v")
        {
            // vertex
            verticesOriginal.push_back(IOUtils::readVector<double, 3>(ss));
        }
        else if (word == "vt")
        {
            // vertex texture
            vertexTexturesOriginal.push_back(IOUtils::readVector<float, 2>(ss));
        }
        else if (word == "vn")
        {
            // vertex normal
            vertexNormalsOriginal.push_back(IOUtils::readVector<double, 3>(ss));
        }
        else if (word == "g")
        {
            // group name
        }
        else if (word == "usemtl")
        {
            // add the number of affected triangles of the previous material to
            // nFacesPerMaterial
            if (currentMaterial != "")
                nFacesPerMaterial.push_back(static_cast<unsigned int>(facesOriginal.size()));

            // used material name
            ss >> word;
            // add the next material to the vector of material names
            currentMaterial = word;
            renderMaterialNames.push_back(currentMaterial);

        }
        else if (word == "s")
        {
            // smoothing
        }
        else if (word == "f")
        {
            // face
            Face f;
            Face fVt;

            // read strings of the form
            //   <v>/<vt>/<vn> <v>/<vt>/<vn> <v>/<vt>/<vn> ...
            //   <v>//<vn> <v>//<vn> <v>//<vn> ...
            //   <v>/<vt>/ <v>/<vt>/ <v>/<vt>/ ...
            //   <v> <v> <v> ...
            std::vector<std::string> indexStrings;
            while (ss >> word)
            {
                indexStrings.push_back(word);
            }
            if (indexStrings.size() < 3)
            {
                std::cout << "Detected element with less than three indices. " <<
                             "It is skipped (found " << indexStrings.size() << " indices).\n";
                continue;
            }
            unsigned int lastIndexPrevious = 0;
            for (size_t i = 0; i < indexStrings.size(); ++i)
            {
                size_t faceIndex = static_cast<size_t>(std::min(static_cast<size_t>(2), i));
                std::string vertexIndex;
                std::stringstream ssVertexIndex(indexStrings[i]);

                // vertex index
                std::getline(ssVertexIndex, vertexIndex, '/');
                f[faceIndex] = static_cast<unsigned int>(std::stoi(vertexIndex)) - 1;

                // vertex texture index
                std::getline(ssVertexIndex, vertexIndex, '/');
                if (vertexIndex != "")
                    fVt[faceIndex] = static_cast<unsigned int>(std::stoi(vertexIndex)) - 1;
                else
                    fVt[faceIndex] = std::numeric_limits<unsigned int>::max();

                // Last value are the vertex normals indices which should always be
                // equal to the face vertex indices.
                if (std::getline(ssVertexIndex, vertexIndex))
                {
                    if (vertexIndex != "" && std::stoi(vertexIndex)-1 != f[faceIndex])
                        std::cout << "vertex id != normal id\n";

                }

                if (i >= 2)
                {
                    if (i >= 3)
                        f[1] = lastIndexPrevious;

                    faceTuples.push_back(std::make_tuple(f, fVt, currentMaterial));
                    for (std::size_t j = 0; j < 3; ++j)
                    {
                        vertexSet.insert(std::make_tuple(f[j], fVt[j], currentMaterial));
                    }
                    facesOriginal.push_back(f);

                    lastIndexPrevious = f[2];
                }

            }

        }
        else if (word == "mtllib")
        {
            std::cout << word << "\n";
            std::cout << "MTLLIB = " <<  ss.str() << "\n";
            ss >> word;
            std::cout << word << "\n";
            mtllib = word;
        }

    }
    in.close();

    // At the end, add the last material to the vector of material names.
    if (currentMaterial != "")
        nFacesPerMaterial.push_back(static_cast<unsigned int>(facesOriginal.size()));

    if (vertexNormalsOriginal.empty())
    {
        ModelUtils::calculateNormals<double>(
                    verticesOriginal,
                    facesOriginal,
                    vertexNormalsOriginal);
    }

    if (verticesOriginal.size() != vertexNormalsOriginal.size())
    {
        std::cout << "#vertices != #vertexNormals, this is not supported."
                     " Vertex normals are ignored.\n";
    }

    // create vertices from vertexSet
    std::vector<std::tuple<unsigned int, unsigned int, std::string>> vertexTuples;
    vertexTuples.resize(vertexSet.size());
    std::move(vertexSet.begin(), vertexSet.end(), vertexTuples.begin());

//    for (const std::tuple<unsigned int, unsigned int, std::string>& t : vertexTuples)
//        std::cout << std::get<0>(t) << ", "
//                  << std::get<1>(t) << ", "
//                  << std::get<2>(t) << "\n";


    Vectors verticesExtended; // non-duplicated
    std::vector<Eigen::Vector2f> vertexTexturesExtended;
    Vectors vertexNormalsExtended; // non-duplicated

    verticesExtended.reserve(vertexTuples.size());
    vertexTexturesExtended.reserve(vertexTuples.size());
    vertexNormalsExtended.reserve(vertexTuples.size());
    std::shared_ptr<VectorIndexMapping> mapping =
            std::make_shared<VectorIndexMapping>(verticesOriginal.size());

    for (std::size_t i = 0; i < vertexTuples.size(); ++i)
    {
        const std::tuple<unsigned int, unsigned int, std::string>& t =
                vertexTuples[i];
        verticesExtended.push_back(verticesOriginal[std::get<0>(t)]);

        if (std::get<1>(t) != std::numeric_limits<unsigned int>::max())
            vertexTexturesExtended.push_back(vertexTexturesOriginal[std::get<1>(t)]);
        vertexNormalsExtended.push_back(vertexNormalsOriginal[std::get<0>(t)]);

        // if previous original vertex is equal to current, insert a duplicate
        unsigned int previousIndex = std::get<0>(vertexTuples[i-1]);
        unsigned int currentIndex = std::get<0>(vertexTuples[i]);
        if (i > 0 && currentIndex == previousIndex)
        {

            mapping->duplicateIndex(currentIndex);
        }
    }

    // create map (vertex, material_name) -> vertex_extended
    std::map<std::tuple<unsigned int, std::string>, unsigned int> map;

    unsigned int duplicateCounter = 0;
    for (std::size_t i = 0; i < vertexTuples.size(); ++i)
    {
        const std::tuple<unsigned int, unsigned int, std::string>& t =
                vertexTuples[i];

        // if previous original vertex is equal to current, we have a duplicate
        unsigned int previousIndex = std::get<0>(vertexTuples[i-1]);
        unsigned int currentIndex = std::get<0>(vertexTuples[i]);
        if (i > 0 && currentIndex == previousIndex)
        {
            ++duplicateCounter;
        }
        else
        {
            duplicateCounter = 0;
        }
        map[std::make_tuple(std::get<0>(t), std::get<2>(t))] =
                static_cast<unsigned int>(mapping->getStartingExtendedIndex(std::get<0>(t)))
                + duplicateCounter;
    }

    for (size_t i = 0; i < faceTuples.size(); ++i)
    {
        const std::tuple<Face, Face, std::string>& tuple = faceTuples[i];
        Face fExtended;
        for (size_t j = 0; j < 3; ++j)
        {
            fExtended[j] = map[std::make_tuple(std::get<0>(tuple)[j], std::get<2>(tuple))];
        }
        facesExtended.push_back(fExtended);
    }

    // assign materials
    std::vector<std::shared_ptr<Appearance>> appearances;
    if (mtllib != "")
    {
        File directory = file;
        directory.goUpDirectory();
        appearances = readMtllib(File(directory.getRelativePath() + File::SEPARATOR + mtllib));
    }

    std::set<std::string> renderMaterialNamesSet;
    for (const std::string& s : renderMaterialNames)
        renderMaterialNamesSet.insert(s);
    if (appearances.size() < renderMaterialNamesSet.size())
    {
        std::cout << "Fewer materials definitions in .mat file than expected. "
                     "Non defined materials are ignored:\n";


    }
    else if (appearances.size() > renderMaterialNamesSet.size())
    {
        std::cout << "Warning: .mat file contains unused materials.\n";
    }
    if (appearances.size() < renderMaterialNamesSet.size() ||
        appearances.size() > renderMaterialNamesSet.size())
    {
        std::cout << "Given materials:\n";
        for (const std::shared_ptr<Appearance>& appearance : appearances)
        {
            std::cout << "  -> " << appearance->getName() << "\n";
        }
        std::cout << "\n";
        std::cout << "Required materials:\n";
        for (const std::string& s : renderMaterialNamesSet)
        {
            std::cout << "  -> " << s << "\n";
        }
    }


    bool triangleWithoutAppearance = false;
    bool materialWithoutTriangles = false;

    std::shared_ptr<Appearances> appearancesReal = std::make_shared<Appearances>();
    for (std::size_t i = 0; i < renderMaterialNames.size(); ++i)
    {
        // find number of triangles
        unsigned int nTriangles = 0;
        std::shared_ptr<Appearance> appearance;
        for (const std::shared_ptr<Appearance>& a : appearances)
        {
            if (renderMaterialNames[i] == a->getName())
            {
                nTriangles = nFacesPerMaterial[i];
                if (i > 0)
                    nTriangles -= nFacesPerMaterial[i-1];
                appearance = a;
            }
        }
        // if there is no material specified for the triangles,
        // use a standard material.
        if (!appearance)
            triangleWithoutAppearance = true;

        if (nTriangles == 0)
            materialWithoutTriangles = true;
        else
            appearancesReal->addAppearance(appearance, nTriangles);
    }

    if (triangleWithoutAppearance)
        std::cout << "Warning: obj contains triangles without material.\n";

    if (materialWithoutTriangles)
        std::cout << "Warning: obj contains materials without triangles.\n";

//    for (int i = 0; i < facesOriginal.size(); ++i)
//    {
//        std::cout << "(" << facesOriginal[i][0] << ", "
//                  << facesOriginal[i][1] << ", "
//                  << facesOriginal[i][2] << "), ("
//                  << facesExtended[i][0] << ", "
//                  << facesExtended[i][1] << ", "
//                  << facesExtended[i][2] << ")\n";
//    }

    std::cout << "Vectors = " << verticesOriginal.size() << "\n"
              << "Vertex Textures = " << vertexTexturesOriginal.size() << "\n"
              << "Vertex Normals = " << vertexNormalsOriginal.size() << "\n"
              << "Faces = " << facesOriginal.size() << "\n"
              << "Faces Vertex Textures = " << vertexTuples.size() << "\n"
              << "Vertices original = " << mapping->getOriginalSize() << "\n"
              << "Vertices extended = " << mapping->getExtendedSize() << "\n";

    std::shared_ptr<Polygon2D> poly2 = std::make_shared<Polygon2D>(verticesOriginal,
                                                                   vertexNormalsOriginal,
                                                                   facesOriginal);

    std::shared_ptr<PolygonRenderModel> renderModel =
            std::make_shared<PolygonRenderModel>(
                ac->getRenderModelManager(), poly2);

    std::shared_ptr<PolygonIndexMapping> pim =
            std::make_shared<PolygonIndexMapping>(mapping, facesExtended);

    renderModel->setPolygonIndexMapping(pim);
    renderModel->setAppearances(appearancesReal);
    renderModel->setTexturingEnabled(true);
    renderModel->reset();

    if (nodeName == "")
        nodeName = file.getName();

    SGLeafNode* node = new SGLeafNode(nodeName);
    std::shared_ptr<SceneLeafData> data = std::make_shared<SceneLeafData>(node);
    node->setData(data);

    data->setGeometricData(poly2);
    data->setRenderModel(renderModel);

//    std::cout << mapping->toString() << "\n";

    return node;
}
