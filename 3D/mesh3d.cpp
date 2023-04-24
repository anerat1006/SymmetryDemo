#include "mesh3d.h"
#include <iostream>
#include <QVector3D>
#include "constants.h"
#include <glm/glm/glm.hpp>

Mesh3D::Mesh3D()
{
}

Mesh3D::~Mesh3D() {
}

void Mesh3D::meshRescale(double factor) {
    Eigen::Vector3d c=mesh.GetCenter();
    mesh.Scale(factor,c);

    mesh.ComputeVertexNormals();

    vertices.clear();

    int i;
    for(i=0;i<mesh.vertices_.size();i++) {
        vertices.push_back(mesh.vertices_[i].x()-c[0]);
        vertices.push_back(mesh.vertices_[i].y()-c[1]);
        vertices.push_back(mesh.vertices_[i].z()-c[2]);
        vertices.push_back(mesh.vertex_normals_[i].x());
        vertices.push_back(mesh.vertex_normals_[i].y());
        vertices.push_back(mesh.vertex_normals_[i].z());
    }

    open3d::geometry::AxisAlignedBoundingBox bbox=mesh.GetAxisAlignedBoundingBox();

    minX=bbox.min_bound_.x();
    minY=bbox.min_bound_.y();
    minZ=bbox.min_bound_.z();

    maxX=bbox.max_bound_.x();
    maxY=bbox.max_bound_.y();
    maxZ=bbox.max_bound_.z();

    auto var=mesh.ComputeConvexHull();

    convHull=open3d::geometry::TriangleMesh(*std::get<0>(var));

    convHull.ComputeVertexNormals();

    convVertices.clear();
    convIndices.clear();

    for(i=0;i<convHull.vertices_.size();i++) {
        convVertices.push_back(convHull.vertices_[i].x() - c[0]);
        convVertices.push_back(convHull.vertices_[i].y() - c[1]);
        convVertices.push_back(convHull.vertices_[i].z() - c[2]);
        convVertices.push_back(convHull.vertex_normals_[i].x());
        convVertices.push_back(convHull.vertex_normals_[i].y());
        convVertices.push_back(convHull.vertex_normals_[i].z());
    }

    for(i=0;i<convHull.triangles_.size();i++) {
        convIndices.push_back(convHull.triangles_[i].x());
        convIndices.push_back(convHull.triangles_[i].y());
        convIndices.push_back(convHull.triangles_[i].z());
    }
}

void Mesh3D::meshRotate(double phi, double theta) {
    int i;
    Eigen::Vector3d c(0,0,0);

    mesh=origMesh;

    float t=glm::radians(theta),p=glm::radians(phi);
    Eigen::Matrix3d Rx=mesh.GetRotationMatrixFromXYZ(Eigen::Vector3d(p,0,0)),Ry=mesh.GetRotationMatrixFromXYZ(Eigen::Vector3d(0,t,0));
    Eigen::Matrix3d R=Ry * Rx;
    mesh.Rotate(R,c);

    mesh.ComputeVertexNormals();

    vertices.clear();

    for(i=0;i<mesh.vertices_.size();i++) {
        vertices.push_back(mesh.vertices_[i].x() - c[0]);
        vertices.push_back(mesh.vertices_[i].y() - c[1]);
        vertices.push_back(mesh.vertices_[i].z() - c[2]);
        vertices.push_back(mesh.vertex_normals_[i].x());
        vertices.push_back(mesh.vertex_normals_[i].y());
        vertices.push_back(mesh.vertex_normals_[i].z());
    }

    open3d::geometry::AxisAlignedBoundingBox bbox=mesh.GetAxisAlignedBoundingBox();

    minX=bbox.min_bound_.x();
    minY=bbox.min_bound_.y();
    minZ=bbox.min_bound_.z();

    maxX=bbox.max_bound_.x();
    maxY=bbox.max_bound_.y();
    maxZ=bbox.max_bound_.z();

    auto var=mesh.ComputeConvexHull();

    convHull=open3d::geometry::TriangleMesh(*std::get<0>(var));

    convHull.ComputeVertexNormals();

    convVertices.clear();
    convIndices.clear();

    for(i=0;i<convHull.vertices_.size();i++) {
        convVertices.push_back(convHull.vertices_[i].x() - c[0]);
        convVertices.push_back(convHull.vertices_[i].y() - c[1]);
        convVertices.push_back(convHull.vertices_[i].z() - c[2]);
        convVertices.push_back(convHull.vertex_normals_[i].x());
        convVertices.push_back(convHull.vertex_normals_[i].y());
        convVertices.push_back(convHull.vertex_normals_[i].z());
    }

    for(i=0;i<convHull.triangles_.size();i++) {
        convIndices.push_back(convHull.triangles_[i].x());
        convIndices.push_back(convHull.triangles_[i].y());
        convIndices.push_back(convHull.triangles_[i].z());
    }
}

void Mesh3D::loadTriangleMeshAssimp(QString fileName, float baseScale) {
    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(fileName.toStdString(), aiProcess_Triangulate);

    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
    {
        std::cout << "ERROR::ASSIMP::" << importer.GetErrorString() << std::endl;
        return;
    }

    if(scene->mNumMeshes > 1) {
        std::cout << "ERROR: Only one mesh alowed!" << std::endl;
        return;
    }

    std::vector<Eigen::Vector3d> verts;
    std::vector<Eigen::Vector3i> trians;
    Eigen::Vector3d center(0,0,0);

    int i,j;
    for(i=0;i<scene->mNumMeshes;i++) {
         aiMesh *msh = scene->mMeshes[i];
         for(j=0;j<msh->mNumVertices;j++) {
             Eigen::Vector3d vec(msh->mVertices[j].x, msh->mVertices[j].y, msh->mVertices[j].z);
             center+=vec;
             verts.push_back(vec);
         }

         for(j=0;j<msh->mNumFaces;j++) {
             aiFace face = msh->mFaces[j];
             if(face.mNumIndices > 3) {
                 std::cout << "ERROR: Only triangles allowed!" << std::endl;
                 return;
             }

             Eigen::Vector3i trian(face.mIndices[0], face.mIndices[1],face.mIndices[2]);
             trians.push_back(trian);
         }
    }

    center /= (double)verts.size();

    for(i=0;i<verts.size();i++)
        verts[i] -= center;

    mesh=open3d::geometry::TriangleMesh(verts,trians);

    Eigen::Vector3d c=mesh.GetCenter();

    open3d::geometry::AxisAlignedBoundingBox bbox=mesh.GetAxisAlignedBoundingBox();
    double maxDim=bbox.max_bound_.x() - bbox.min_bound_.x();
    if(maxDim < bbox.max_bound_.y() - bbox.min_bound_.y())
        maxDim=bbox.max_bound_.y() - bbox.min_bound_.y();
    if(maxDim < bbox.max_bound_.z() - bbox.min_bound_.z())
        maxDim=bbox.max_bound_.z() - bbox.min_bound_.z();

    baseScale=100.0 / maxDim;

    mesh.Scale(baseScale,c);
    mesh.OrientTriangles();

    mesh.ComputeVertexNormals();

    origMesh=mesh;

    auto var=mesh.ComputeConvexHull();

    convHull=open3d::geometry::TriangleMesh(*std::get<0>(var));
    convHull.OrientTriangles();

    convHull.ComputeVertexNormals();

    vertices.clear();
    indices.clear();

    for(i=0;i<mesh.vertices_.size();i++) {
        vertices.push_back(mesh.vertices_[i].x() - c[0]);
        vertices.push_back(mesh.vertices_[i].y() - c[1]);
        vertices.push_back(mesh.vertices_[i].z() - c[2]);
        vertices.push_back(mesh.vertex_normals_[i].x());
        vertices.push_back(mesh.vertex_normals_[i].y());
        vertices.push_back(mesh.vertex_normals_[i].z());
    }

    bbox=mesh.GetAxisAlignedBoundingBox();

    minX=bbox.min_bound_.x();
    minY=bbox.min_bound_.y();
    minZ=bbox.min_bound_.z();

    maxX=bbox.max_bound_.x();
    maxY=bbox.max_bound_.y();
    maxZ=bbox.max_bound_.z();

    for(i=0;i<mesh.triangles_.size();i++) {
        indices.push_back(mesh.triangles_[i].x());
        indices.push_back(mesh.triangles_[i].y());
        indices.push_back(mesh.triangles_[i].z());
    }

    convVertices.clear();
    convIndices.clear();

    for(i=0;i<convHull.vertices_.size();i++) {
        convVertices.push_back(convHull.vertices_[i].x() - c[0]);
        convVertices.push_back(convHull.vertices_[i].y() - c[1]);
        convVertices.push_back(convHull.vertices_[i].z() - c[2]);
        convVertices.push_back(convHull.vertex_normals_[i].x());
        convVertices.push_back(convHull.vertex_normals_[i].y());
        convVertices.push_back(convHull.vertex_normals_[i].z());
    }

    for(i=0;i<convHull.triangles_.size();i++) {
        convIndices.push_back(convHull.triangles_[i].x());
        convIndices.push_back(convHull.triangles_[i].y());
        convIndices.push_back(convHull.triangles_[i].z());
    }
}

void Mesh3D::loadTriangleMesh(QString fileName, float baseScale) {
    open3d::io::ReadTriangleMesh(fileName.toStdString(),mesh);

    Eigen::Vector3d c=mesh.GetCenter();
    mesh.Scale(baseScale,c);

    Eigen::Matrix3d R=mesh.GetRotationMatrixFromXYZ(Eigen::Vector3d(0,PI/4.0,0));
    //mesh.Rotate(R,c);

    mesh.ComputeVertexNormals();

    int i;
    for(i=0;i<mesh.vertices_.size();i++) {
        vertices.push_back(mesh.vertices_[i].x() - c[0]);
        vertices.push_back(mesh.vertices_[i].y() - c[1]);
        vertices.push_back(mesh.vertices_[i].z() - c[2]);
        vertices.push_back(mesh.vertex_normals_[i].x());
        vertices.push_back(mesh.vertex_normals_[i].y());
        vertices.push_back(mesh.vertex_normals_[i].z());
    }

    open3d::geometry::AxisAlignedBoundingBox bbox=mesh.GetAxisAlignedBoundingBox();

    minX=bbox.min_bound_.x();
    minY=bbox.min_bound_.y();
    minZ=bbox.min_bound_.z();

    maxX=bbox.max_bound_.x();
    maxY=bbox.max_bound_.y();
    maxZ=bbox.max_bound_.z();

    for(i=0;i<mesh.triangles_.size();i++) {
        indices.push_back(mesh.triangles_[i].x());
        indices.push_back(mesh.triangles_[i].y());
        indices.push_back(mesh.triangles_[i].z());
    }
}

std::vector<float> Mesh3D::getVertex_Normals()
{
    return vertices;
}

std::vector<unsigned int> Mesh3D::getFaceIndices()
{
    return indices;
}

open3d::geometry::TriangleMesh *Mesh3D::getMesh()
{
    return &mesh;
}

open3d::geometry::TriangleMesh *Mesh3D::getConvHullMesh()
{
    return &convHull;
}

std::vector<float> Mesh3D::getConvVertices()
{
    return convVertices;
}

std::vector<unsigned int> Mesh3D::getConvIndices()
{
    return convIndices;
}

float Mesh3D::getMaxZ() const
{
    return maxZ;
}

float Mesh3D::getMaxY() const
{
    return maxY;
}

float Mesh3D::getMaxX() const
{
    return maxX;
}

float Mesh3D::getMinZ() const
{
    return minZ;
}

float Mesh3D::getMinY() const
{
    return minY;
}

float Mesh3D::getMinX() const
{
    return minX;
}
