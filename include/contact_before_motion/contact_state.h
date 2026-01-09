#pragma once

#include <cnoid/EigenUtil>

namespace contact_before_motion{
  class ContactCandidate {
  public:
    ContactCandidate() {}
    ContactCandidate(std::string bodyName_, std::string linkName_, cnoid::Isometry3 localPose_=cnoid::Isometry3::Identity(), bool isStatic_=true) : bodyName(bodyName_), linkName(linkName_), localPose(localPose_), isStatic(isStatic_) {}
    std::string bodyName;
    std::string linkName;
    cnoid::Isometry3 localPose = cnoid::Isometry3::Identity();
    bool isStatic=true;
  };
  class Contact {
    friend bool operator==(const Contact& a, const Contact& b) { // ContactはlocalPose違いも同じ接触とみなす. 終了条件等で複数のcontactをまとめて扱うため. 別の接触とみなしてほしい場合はnameを別にすること.
      return ((a.c1.bodyName == b.c1.bodyName) && (a.c2.bodyName == b.c2.bodyName) && (a.c1.linkName == b.c1.linkName) && (a.c2.linkName == b.c2.linkName)) ||
        ((a.c1.bodyName == b.c2.bodyName) && (a.c2.bodyName == b.c1.bodyName) && (a.c1.linkName == b.c2.linkName) && (a.c2.linkName == b.c1.linkName));
    }
  public:
    Contact() {}
    Contact(ContactCandidate c1_, ContactCandidate c2_) : c1(c1_), c2(c2_) {}
    ContactCandidate c1;
    ContactCandidate c2;
  };
  class ContactState {
    friend bool operator==(const ContactState& a, const ContactState& b) { // ContactStateはlocalPoseのrotationの違いのみ無視する. rotationは接触時にZ方向を無視するため.リンクごとの接触を行うため、staticと接触している場合はリンク内の違いは無視する. graphとして再訪を防ぐために比較する.
      if (a.contacts.size() != b.contacts.size()) return false; // 数が違う
      for (int i=0;i<a.contacts.size();i++) {
        bool has_a = false;
        for (int j=0;(j<b.contacts.size()) && !has_a; j++) {
          if (!(a.contacts[i] == b.contacts[j])) continue; // 名前が違う
          if (!((a.contacts[i].c1.linkName == b.contacts[j].c1.linkName) && a.contacts[i].c1.localPose.isApprox(b.contacts[j].c1.localPose) && (a.contacts[i].c2.linkName == b.contacts[j].c2.linkName) && a.contacts[i].c2.localPose.isApprox(b.contacts[j].c2.localPose)) ||
              ((a.contacts[i].c1.linkName == b.contacts[j].c2.linkName) && a.contacts[i].c1.localPose.isApprox(b.contacts[j].c2.localPose) && (a.contacts[i].c2.linkName == b.contacts[j].c1.linkName) && a.contacts[i].c2.localPose.isApprox(b.contacts[j].c1.localPose))) continue;
          has_a = true;
        }
        if (!has_a) return false;
      }
      return true;
    }
  public:
    std::vector<std::vector<double> > transition;
    std::vector<double> frame;
    std::vector<Contact> contacts;
  };
}
