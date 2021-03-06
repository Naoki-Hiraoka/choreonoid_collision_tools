#include <choreonoid_qhull/choreonoid_qhull.h>

#include <iostream>

#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <cnoid/EigenUtil>

#include <qhulleigen/qhulleigen.h>

namespace choreonoid_qhull{
  void addMesh(cnoid::SgMeshPtr model, std::shared_ptr<cnoid::MeshExtractor> meshExtractor){
    cnoid::SgMeshPtr mesh = meshExtractor->currentMesh();
    const cnoid::Affine3& T = meshExtractor->currentTransform();

    const int vertexIndexTop = model->getOrCreateVertices()->size();

    const cnoid::SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = vertices.size();
    for(int i=0; i < numVertices; ++i){
      const cnoid::Vector3 v = T * vertices[i].cast<cnoid::Affine3::Scalar>();
      model->vertices()->push_back(v.cast<cnoid::Vector3f::Scalar>());
    }

    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
      cnoid::SgMesh::TriangleRef tri = mesh->triangle(i);
      const int v0 = vertexIndexTop + tri[0];
      const int v1 = vertexIndexTop + tri[1];
      const int v2 = vertexIndexTop + tri[2];
      model->addTriangle(v0, v1, v2);
    }
  }

  cnoid::SgMeshPtr convertToSgMesh (const cnoid::SgNodePtr collisionshape){

    if (!collisionshape) return nullptr;

    std::shared_ptr<cnoid::MeshExtractor> meshExtractor = std::make_shared<cnoid::MeshExtractor>();
    cnoid::SgMeshPtr model = new cnoid::SgMesh;
    if(meshExtractor->extract(collisionshape, [&]() { addMesh(model,meshExtractor); })){
      model->setName(collisionshape->name());
    }else{
      std::cerr << "[convertToSgMesh] meshExtractor->extract failed " << collisionshape->name() << std::endl;
      return nullptr;
    }

    return model;
  }

  cnoid::SgNodePtr convertToConvexHull(const cnoid::SgNodePtr collisionshape) {
    cnoid::SgMeshPtr model = convertToSgMesh(collisionshape);

    if (!model || model->vertices()->size()==0) return nullptr;

    // qhull
    Eigen::MatrixXd vertices(3,model->vertices()->size());
    for(size_t i=0;i<model->vertices()->size();i++){
      vertices.col(i) = model->vertices()->at(i).cast<Eigen::Vector3d::Scalar>();
    }
    Eigen::MatrixXd hull;
    std::vector<std::vector<int> > faces;
    if(!qhulleigen::convexhull(vertices,hull,faces)) return nullptr;

    cnoid::SgMeshPtr coldetModel(new cnoid::SgMesh);
    coldetModel->setName(collisionshape->name());

    coldetModel->getOrCreateVertices()->resize(hull.cols());
    coldetModel->setNumTriangles(faces.size());

    for(size_t i=0;i<hull.cols();i++){
      coldetModel->vertices()->at(i) = hull.col(i).cast<cnoid::Vector3f::Scalar>();
    }

    for(size_t i=0;i<faces.size();i++){
      coldetModel->setTriangle(i, faces[i][0], faces[i][1], faces[i][2]);
    }

    cnoid::SgShapePtr ret(new cnoid::SgShape);
    ret->setMesh(coldetModel);
    ret->setName(collisionshape->name());
    return ret;
  }

  void convertAllCollisionToConvexHull(cnoid::BodyPtr& robot){
    for(size_t i=0;i<robot->numLinks();i++){
      cnoid::SgNodePtr coldetModel = convertToConvexHull(robot->link(i)->collisionShape());
      if(coldetModel){
        robot->link(i)->setCollisionShape(coldetModel);
      }else{
        std::cerr << "[convertCollisionToConvexHull] convex hull " << robot->link(i)->name() << " fail" << std::endl;
      }
    }
  }
}
