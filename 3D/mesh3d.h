#ifndef MESH3D_H
#define MESH3D_H

#include <QString>
#include <QFile>
#include "open3d/Open3D.h"
#include <QVector3D>
#include <QMatrix4x4>
#include "assimp/Importer.hpp"
#include "assimp/scene.h"
#include "assimp/postprocess.h"

class Mesh3D
{
public:
    Mesh3D();

    ~Mesh3D();

    void loadTriangleMesh(QString fileName, float baseScale);

    void loadTriangleMeshAssimp(QString fileName, float baseScale);

    std::vector<float> getVertex_Normals();

    std::vector<unsigned int> getFaceIndices();

    void meshRescale(double factor);

    void meshRotate(double phi, double theta);

    float getMinX() const;

    float getMinY() const;

    float getMinZ() const;

    float getMaxX() const;

    float getMaxY() const;

    float getMaxZ() const;

    open3d::geometry::TriangleMesh *getMesh();

    open3d::geometry::TriangleMesh *getConvHullMesh();

    std::vector<float> getConvVertices();

    std::vector<unsigned int> getConvIndices();

private:
    std::vector<float> vertices,convVertices;
    std::vector<unsigned int> indices, convIndices;

    float minX,minY,minZ;
    float maxX,maxY,maxZ;

    open3d::geometry::TriangleMesh mesh, origMesh, convHull;
};

#endif // MESH3D_H
