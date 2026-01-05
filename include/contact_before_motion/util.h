#pragma once

#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <contact_before_motion/contact_state.h>
#include <choreonoid_contact_candidate_generator/choreonoid_contact_candidate_generator.h>

namespace contact_before_motion{
  enum class IKState
    {
      DETACH_FIXED, // ついている接触を離す.
      ATTACH_PRE, // 触れる直前にまで近づける.
      ATTACH_FIXED, // 離れている接触をつける.
    };
  void convertContactCandidates(const std::vector<choreonoid_contact_candidate_generator::ContactCandidate>& in, std::vector<std::shared_ptr<ContactCandidate> >& out, bool isStatic);
  std::vector<cnoid::SgNodePtr> generateCandidateMarkers(const std::vector<cnoid::BodyPtr>& bodies, const std::vector<std::shared_ptr<ContactCandidate> >& ccs);
  void generateCandidateVisualLink(const std::vector<cnoid::BodyPtr>& bodies, const cnoid::LinkPtr link, const std::vector<std::shared_ptr<ContactCandidate> >& ccs, const cnoid::Vector3f color=cnoid::Vector3f(0.0,1.0,0.0));
}

inline std::ostream &operator<<(std::ostream &os, const contact_before_motion::ContactState& state) {
  for (int i=0; i<state.contacts.size(); i++) {
    os << "contact " << i << std::endl;
    os << "c1 body name : " << state.contacts[i].c1.bodyName << std::endl;
    os << "c1 link name : " << state.contacts[i].c1.linkName << std::endl;
    os << "pos : " << (state.contacts[i].c1.localPose.translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
    os << "rot : " << (state.contacts[i].c1.localPose.linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
    os << "isStatic : " << state.contacts[i].c1.isStatic << std::endl;
    os << "c2 body name : " << state.contacts[i].c2.bodyName << std::endl;
    os << "c2 link name : " << state.contacts[i].c2.linkName << std::endl;
    os << "pos : " << (state.contacts[i].c2.localPose.translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
    os << "rot : " << (state.contacts[i].c2.localPose.linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
    os << "isStatic : " << state.contacts[i].c2.isStatic << std::endl;
    }
  os << std::endl;
  return os;
}
