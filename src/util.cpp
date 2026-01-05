#include <contact_before_motion/contact_graph.h>
#include <cnoid/MeshGenerator>

namespace contact_before_motion{
  void convertContactCandidates(const std::vector<choreonoid_contact_candidate_generator::ContactCandidate>& in, std::vector<std::shared_ptr<ContactCandidate> >& out, bool isStatic) {
    out.clear();
    for (int i=0; i<in.size(); i++) {
      std::shared_ptr<contact_before_motion::ContactCandidate> cc = std::make_shared<contact_before_motion::ContactCandidate>();
      cc->bodyName = in[i].body_name;
      cc->linkName = in[i].link_name;
      cc->localPose.translation() = in[i].p.cast<double>();
      cc->localPose.linear() = in[i].R.cast<double>();
      cc->isStatic = isStatic;
      out.push_back(cc);
    }
  }

  std::vector<cnoid::SgNodePtr> generateCandidateMarkers(const std::vector<cnoid::BodyPtr>& bodies, const std::vector<std::shared_ptr<ContactCandidate> >& ccs) {
    std::vector<cnoid::SgNodePtr> drawOnObjects;
    for(int i=0;i<ccs.size();i++) {
      cnoid::SgLineSetPtr lines = new cnoid::SgLineSet;
      lines->setLineWidth(5.0);
      lines->getOrCreateColors()->resize(3);
      lines->getOrCreateColors()->at(0) = cnoid::Vector3f(1.0,0.0,0.0);
      lines->getOrCreateColors()->at(1) = cnoid::Vector3f(0.0,1.0,0.0);
      lines->getOrCreateColors()->at(2) = cnoid::Vector3f(0.0,0.0,1.0);
      lines->getOrCreateVertices()->resize(4);
      lines->colorIndices().resize(0);
      lines->addLine(0,1); lines->colorIndices().push_back(0); lines->colorIndices().push_back(0);
      lines->addLine(0,2); lines->colorIndices().push_back(1); lines->colorIndices().push_back(1);
      lines->addLine(0,3); lines->colorIndices().push_back(2); lines->colorIndices().push_back(2);
      cnoid::Isometry3 pose = ccs[i]->localPose; // bodyを持たない場合
      for (int j=0; j<bodies.size(); j++) {
        if (bodies[j]->name() != ccs[i]->bodyName) continue;
        if(bodies[j]->joint(ccs[i]->linkName)) {
          pose = bodies[j]->joint(ccs[i]->linkName)->T() * ccs[i]->localPose;
        }
      }
      lines->getOrCreateVertices()->at(0) = pose.translation().cast<cnoid::Vector3f::Scalar>();
      lines->getOrCreateVertices()->at(1) = (pose * (0.05 * cnoid::Vector3::UnitX())).cast<cnoid::Vector3f::Scalar>();
      lines->getOrCreateVertices()->at(2) = (pose * (0.05 * cnoid::Vector3::UnitY())).cast<cnoid::Vector3f::Scalar>();
      lines->getOrCreateVertices()->at(3) = (pose * (0.05 * cnoid::Vector3::UnitZ())).cast<cnoid::Vector3f::Scalar>();
      drawOnObjects.push_back(lines);
    }
    return drawOnObjects;
  }

  void generateCandidateVisualLink(const std::vector<cnoid::BodyPtr>& bodies, const cnoid::LinkPtr link, const std::vector<std::shared_ptr<ContactCandidate> >& ccs, const cnoid::Vector3f color) {
    cnoid::MeshGenerator meshGenerator;
    for(int i=0;i<ccs.size();i++) {
      cnoid::SgShapePtr shape = new cnoid::SgShape();
      shape->setMesh(meshGenerator.generateSphere(0.025));
      cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
      material->setTransparency(0);
      material->setDiffuseColor(color);
      shape->setMaterial(material);
      cnoid::Isometry3 pose = ccs[i]->localPose; // bodyを持たない場合
      for (int j=0; j<bodies.size(); j++) {
        if (bodies[j]->name() != ccs[i]->bodyName) continue;
        if(bodies[j]->link(ccs[i]->linkName)) {
          pose = bodies[j]->link(ccs[i]->linkName)->T() * ccs[i]->localPose;
        }
      }
      cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform(pose);
      posTransform->addChild(shape);
      link->addShapeNode(posTransform);
    }
  }
}
