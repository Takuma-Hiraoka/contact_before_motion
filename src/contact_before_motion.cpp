#include <contact_before_motion/contact_graph.h>
#include <contact_before_motion/contact_node.h>
#include <contact_before_motion/util.h>
#include <ik_constraint2_scfr/ik_constraint2_scfr.h>
#include <limits>
#include <random>
#include <unordered_set>

namespace contact_before_motion{
  inline std::set<cnoid::BodyPtr> getBodies(const std::vector<cnoid::LinkPtr>& links){
    std::set<cnoid::BodyPtr> bodies;
    for(size_t i=0;i<links.size();i++){
      if(links[i]->body()) bodies.insert(links[i]->body());
    }
    return bodies;
  }

  inline ContactStateKey toKey(const ContactState& state)
  {
    ContactStateKey key;
    key.contacts.reserve(state.contacts.size());

    for (int i=0; i<state.contacts.size(); i++){
      ContactKey ck;

      // 名前とリンク名のペアを hash 化（順番違いも吸収）
      std::size_t h1 = std::hash<std::string>()(state.contacts[i].c1.bodyName + "#" + state.contacts[i].c1.linkName);
      std::size_t h2 = std::hash<std::string>()(state.contacts[i].c2.bodyName + "#" + state.contacts[i].c2.linkName);

      if (h1 < h2) {
        ck.id1 = h1;
        ck.id2 = h2;

        cnoid::Vector3 t1 = state.contacts[i].c1.localPose.translation();
        cnoid::Vector3 t2 = state.contacts[i].c2.localPose.translation();

        ck.x1 = t1[0]; ck.y1 = t1[1]; ck.z1 = t1[2];
        ck.x2 = t2[0]; ck.y2 = t2[1]; ck.z2 = t2[2];
      } else {
        ck.id1 = h2;
        ck.id2 = h1;

        cnoid::Vector3 t1 = state.contacts[i].c2.localPose.translation();
        cnoid::Vector3 t2 = state.contacts[i].c1.localPose.translation();

        ck.x1 = t1[0]; ck.y1 = t1[1]; ck.z1 = t1[2];
        ck.x2 = t2[0]; ck.y2 = t2[1]; ck.z2 = t2[2];
      }

      key.contacts.push_back(ck);
    }

    // ContactState の contacts 順不同を吸収
    std::sort(key.contacts.begin(), key.contacts.end(),
              [](ContactKey& a, ContactKey& b){
                return std::tie(a.id1, a.id2, a.x1, a.y1, a.z1, a.x2, a.y2, a.z2)
                  < std::tie(b.id1, b.id2, b.x1, b.y1, b.z1, b.x2, b.y2, b.z2);
              });

    return key;
  }

  bool WholeBodyContactPlanner::solve() {
    std::shared_ptr<ContactNode> current_node = std::make_shared<ContactNode>();
    if (this->currentContactState->transition.size() == 0) this->currentContactState->transition.push_back(this->currentContactState->frame);
    current_node->state() = *(this->currentContactState);
    this->graph().push_back(current_node);
    this->graph_states.reserve(std::numeric_limits<int>::max());
    for (int i=0;i<this->graph().size();i++) {
      this->graph_states.insert(toKey(std::static_pointer_cast<ContactNode>(this->graph()[i])->state()));
    }

    return this->search();
  }

  std::shared_ptr<graph_search::Planner::TransitionCheckParam> WholeBodyContactPlanner::generateCheckParam() {
    std::shared_ptr<ContactTransitionCheckParam> checkParam = std::make_shared<ContactTransitionCheckParam>();
    cloneCheckParam(checkParam);
    return checkParam;
  }

  void WholeBodyContactPlanner::cloneCheckParam(std::shared_ptr<ContactTransitionCheckParam> checkParam) {
    std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
    checkParam->bodies = std::vector<cnoid::BodyPtr>(this->bodies.size());
    for(int b=0; b<this->bodies.size(); b++){
      checkParam->bodies[b] = this->bodies[b]->clone();
      modelMap[this->bodies[b]] = checkParam->bodies[b];
      if (this->pikParam.viewer) this->pikParam.viewer->objects(checkParam->bodies[b]);
    }
    checkParam->variables = std::vector<cnoid::LinkPtr>(this->variables.size());
    for(int v=0;v<this->variables.size();v++){
      checkParam->variables[v] = modelMap[this->variables[v]->body()]->link(this->variables[v]->index());
    }
    checkParam->constraints = std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(this->constraints.size());
    for(int j=0;j<this->constraints.size();j++){
      checkParam->constraints[j] = this->constraints[j]->clone(modelMap);
    }
    checkParam->rejections = std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(this->rejections.size());
    for(int j=0;j<this->rejections.size();j++){
      checkParam->rejections[j] = this->rejections[j]->clone(modelMap);
    }
    checkParam->nominals = std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(this->nominals.size());
    for(int j=0;j<this->nominals.size();j++){
      checkParam->nominals[j] = this->nominals[j]->clone(modelMap);
    }
    checkParam->goalConstraint = this->goalConstraint->clone(modelMap);
    checkParam->contactDynamicCandidates = this->contactDynamicCandidates;
    checkParam->contactStaticCandidates = this->contactStaticCandidates;
    checkParam->pikParam = this->pikParam;
    checkParam->gikParam = this->gikParam;
    checkParam->debugLevel = this->debugLevel();
    checkParam->addCandidateDistance = this->addCandidateDistance;
  }

  void WholeBodyContactPlanner::preCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> extend_node) {
    std::weak_ptr<graph_search::Node> parent = extend_node->parent();
    std::shared_ptr<ContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam);
    if (parent.expired()) contactCheckParam->preState = std::static_pointer_cast<ContactNode>(extend_node)->state(); // 初期状態なので絶対に遷移可能にしておく.
    else contactCheckParam->preState = std::static_pointer_cast<ContactNode>(parent.lock())->state();
    contactCheckParam->postState = std::static_pointer_cast<ContactNode>(extend_node)->state();
  }

  void WholeBodyContactPlanner::postCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> extend_node) {
    std::static_pointer_cast<ContactNode>(extend_node)->state() = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam)->postState;
  }

  bool WholeBodyContactPlanner::checkTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) {
    std::shared_ptr<ContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam);
    return this->checkTransitionImpl(contactCheckParam,
                                     contactCheckParam->postState);
  }

  bool WholeBodyContactPlanner::isGoalSatisfied(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) {
    std::shared_ptr<ContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam);

    contactCheckParam->goalConstraint->updateBounds();
    return contactCheckParam->goalConstraint->isSatisfied();
  }

  void WholeBodyContactPlanner::calcHeuristic(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> node) {
    std::shared_ptr<ContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam);
    ContactState state = std::static_pointer_cast<ContactNode>(node)->state();
    std::shared_ptr<ik_constraint2::PositionConstraint> goalConstraint = std::static_pointer_cast<ik_constraint2::PositionConstraint>(contactCheckParam->goalConstraint);
    cnoid::Vector3 pos = cnoid::Vector3::Zero();
    for (int i=0; i<state.contacts.size(); i++) {
      pos += state.contacts[i].c2.localPose.translation();
    }
    cnoid::Vector3 centorOfContact = pos / state.contacts.size();
    node->heuristic() = (centorOfContact - goalConstraint->B_localpos().translation()).norm();
  }

  std::vector<std::shared_ptr<graph_search::Node> > WholeBodyContactPlanner::gatherAdjacentNodes(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) {
    std::shared_ptr<ContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam);
    ContactState extend_state = contactCheckParam->postState;
    if (contactCheckParam->debugLevel >= 2) {
      std::cerr << "extend_state" << std::endl;
      std::cerr << extend_state << std::endl;
    }
    std::vector<std::shared_ptr<graph_search::Node> > adjacentNodes;
    // 接触の減少
    if (extend_state.contacts.size() >= 2) {
      for (int i=0; i<extend_state.contacts.size();i++) {
        std::shared_ptr<ContactNode> newNode = std::make_shared<ContactNode>();
        // frameとcontactsのみコピーする. transitionは使わないうえにすべての子ノードでコピーすると重い
        newNode->state().frame = extend_state.frame;
        newNode->state().contacts = extend_state.contacts;
        newNode->state().contacts.erase(newNode->state().contacts.begin()+i);
        adjacentNodes.push_back(newNode);
      }
    }

    // static contact
    for (int i=0; i<contactCheckParam->contactDynamicCandidates.size(); i++) {
      // staticCandidateと接触しているものを更にstaticCandidateと接触させることはしない
      bool skip = false;
      for (int j=0; j<extend_state.contacts.size(); j++) {
        if (((contactCheckParam->contactDynamicCandidates[i]->bodyName == extend_state.contacts[j].c1.bodyName) && (contactCheckParam->contactDynamicCandidates[i]->linkName == extend_state.contacts[j].c1.linkName) && (extend_state.contacts[j].c2.isStatic)) ||
            ((contactCheckParam->contactDynamicCandidates[i]->bodyName == extend_state.contacts[j].c2.bodyName) && (contactCheckParam->contactDynamicCandidates[i]->linkName == extend_state.contacts[j].c2.linkName) && (extend_state.contacts[j].c1.isStatic))) {
          skip = true;
          break;
        }
      }
      if (skip) continue;

      // ルートリンク位置からaddCandidateDistanceを超える距離のstaticCandidateと接触させることはしない
      // 高速化のため. gikを使うまでもなく解けない
      cnoid::Vector3 rootPos;
      for (int b=0; b<contactCheckParam->bodies.size(); b++) {
        if ((contactCheckParam->bodies[b]->name() == contactCheckParam->contactDynamicCandidates[i]->bodyName) && contactCheckParam->bodies[b]->joint(contactCheckParam->contactDynamicCandidates[i]->linkName)) rootPos = contactCheckParam->bodies[b]->rootLink()->p();
      }

      std::vector<std::shared_ptr<ContactCandidate> > contactStaticCandidatesLimited;
      for (int j=0; j<contactCheckParam->contactStaticCandidates.size(); j++) {
        cnoid::Isometry3 trans = cnoid::Isometry3::Identity();
        for (int b=0; b<contactCheckParam->bodies.size(); b++) {
          if ((contactCheckParam->bodies[b]->name() == contactCheckParam->contactStaticCandidates[j]->bodyName) && contactCheckParam->bodies[b]->joint(contactCheckParam->contactStaticCandidates[j]->linkName)) trans = contactCheckParam->bodies[b]->joint(contactCheckParam->contactStaticCandidates[j]->linkName)->T();
        }
        if ((rootPos - (trans * contactCheckParam->contactStaticCandidates[j]->localPose).translation()).norm() > contactCheckParam->addCandidateDistance) continue;
        contactStaticCandidatesLimited.push_back(contactCheckParam->contactStaticCandidates[j]);
      }

      for (int j=0; j<contactStaticCandidatesLimited.size(); j++) {
        std::shared_ptr<ContactNode> newNode = std::make_shared<ContactNode>();
        // frameとcontactsのみコピーする. transitionは使わないうえにすべての子ノードでコピーすると重い
        newNode->state().frame = extend_state.frame;
        newNode->state().contacts = extend_state.contacts;
        Contact c = Contact(*(contactCheckParam->contactDynamicCandidates[i]), *(contactStaticCandidatesLimited[j]));
        newNode->state().contacts.push_back(c);
        adjacentNodes.push_back(newNode);
      }
    }

    // dynamic contact
    // locomotionにおいては、dynamicとdynamicは接触しない

    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(adjacentNodes.begin(), adjacentNodes.end(), g);
    return adjacentNodes;

  }

  void WholeBodyContactPlanner::addNodes2Graph(std::vector<std::shared_ptr<graph_search::Node> >& nodes) {
    // 再訪しない
    nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
                               [&](const std::shared_ptr<graph_search::Node>& n) {
                                 return graph_states.count(toKey(std::static_pointer_cast<ContactNode>(n)->state())) > 0;
                               }),
                nodes.end());

    for (int i=0;i<nodes.size();i++) {
      graph_states.insert(toKey(std::static_pointer_cast<ContactNode>(nodes[i])->state()));
    }

    graph_search::Planner::addNodes2Graph(nodes);
  }

  bool WholeBodyContactPlanner::checkTransitionImpl(std::shared_ptr<const ContactTransitionCheckParam> checkParam,
                                                    ContactState& postState
                                                    ) {
    if (this->debugLevel() >= 2) {
      std::cerr << "[GraphSearchWholeBodyContactPlanner] checkTransition" << std::endl;
      std::cerr << "preState" << std::endl;
      std::cerr << checkParam->preState << std::endl;
      std::cerr << "postState" << std::endl;
      std::cerr << postState << std::endl;
      getchar();
    }

    global_inverse_kinematics_solver::frame2Link(checkParam->preState.frame, checkParam->variables);
    postState.transition.clear();

    if (postState.contacts.size() > checkParam->preState.contacts.size()) {
      // attach
      if (!solveContactIK(checkParam, postState.contacts.back(), postState, IKState::ATTACH_PRE)) return false;
      if (!solveContactIK(checkParam, postState.contacts.back(), postState, IKState::ATTACH_FIXED)) return false;
    } else if (postState.contacts.size() < checkParam->preState.contacts.size()) {
      // detach
      bool find_detach_contact = false;
      Contact moveContact;
      for(int i=0;i<checkParam->preState.contacts.size() && !find_detach_contact;i++) {
        if(std::find(postState.contacts.begin(), postState.contacts.end(), checkParam->preState.contacts[i]) == postState.contacts.end()) {
          moveContact = checkParam->preState.contacts[i];
          find_detach_contact = true;
        }
      }
      if (!find_detach_contact) {
        std::cerr << "[GraphSearchWholeBodyContactPlanner] checkTransition failed!! cannot find detach contact" << std::endl;
        return false;
      }
      if (!solveContactIK(checkParam, moveContact, postState, IKState::DETACH_FIXED)) return false;
    } else {
      if (checkParam->preState == postState) {
        // 同じ. はじめの接触.
        postState.transition.push_back(checkParam->preState.frame);
        return true;
      }
      std::cerr << "[GraphSearchWholeBodyContactPlanner] checkTransition failed!! postState.contacts.size() is same as preState.contacts.size()" << std::endl;
      return false;
    }
    global_inverse_kinematics_solver::link2Frame(checkParam->variables, postState.frame);
    return true;
  }

  bool WholeBodyContactPlanner::solveContactIK(std::shared_ptr<const ContactTransitionCheckParam> checkParam,
                                               Contact& moveContact,
                                               ContactState& postState,
                                               const IKState& ikState
                                               ) {
    std::shared_ptr<const WholeBodyContactPlanner::ContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<const WholeBodyContactPlanner::ContactTransitionCheckParam>(checkParam);
    std::shared_ptr<std::vector<std::vector<double> > > tmpPath = std::make_shared<std::vector<std::vector<double> > >();
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0; // 幾何干渉や重心制約.
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1; // 動かさない接触
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints2; // 動かす接触
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;

    for (int i=0; i<contactCheckParam->constraints.size(); i++) {
      bool skip=false;
      if (typeid(*(contactCheckParam->constraints[i]))==typeid(ik_constraint2_distance_field::DistanceFieldCollisionConstraint)) {
        std::shared_ptr<ik_constraint2::CollisionConstraint> constraint = std::static_pointer_cast<ik_constraint2::CollisionConstraint>(contactCheckParam->constraints[i]);
        for (int j=0; j<contactCheckParam->preState.contacts.size() && !skip; j++) {
          if (((contactCheckParam->preState.contacts[j].c1.bodyName == constraint->A_link()->body()->name()) && (contactCheckParam->preState.contacts[j].c1.linkName == constraint->A_link()->name()) && contactCheckParam->preState.contacts[j].c2.isStatic) ||
              ((contactCheckParam->preState.contacts[j].c2.bodyName == constraint->A_link()->body()->name()) && (contactCheckParam->preState.contacts[j].c2.linkName == constraint->A_link()->name()) && contactCheckParam->preState.contacts[j].c1.isStatic)) skip = true;
        }
        if (!skip && ((ikState==IKState::ATTACH_FIXED) ||
                      (ikState==IKState::DETACH_FIXED))) {
          if (((moveContact.c1.bodyName == constraint->A_link()->body()->name()) && (moveContact.c1.linkName == constraint->A_link()->name())) ||
              ((moveContact.c2.bodyName == constraint->A_link()->body()->name()) && (moveContact.c2.linkName == constraint->A_link()->name()))) skip = true;
        }
      }
      if (typeid(*(contactCheckParam->constraints[i]))==typeid(ik_constraint2_bullet::BulletCollisionConstraint)) {
        std::shared_ptr<ik_constraint2::CollisionConstraint> constraint = std::static_pointer_cast<ik_constraint2::CollisionConstraint>(contactCheckParam->constraints[i]);
        for (int j=0; j<contactCheckParam->preState.contacts.size() && !skip; j++) {
          if (((contactCheckParam->preState.contacts[j].c1.bodyName == constraint->A_link()->body()->name()) && (contactCheckParam->preState.contacts[j].c1.linkName == constraint->A_link()->name()) && (contactCheckParam->preState.contacts[j].c2.bodyName == constraint->B_link()->body()->name()) && (contactCheckParam->preState.contacts[j].c2.linkName == constraint->B_link()->name())) ||
              ((contactCheckParam->preState.contacts[j].c1.bodyName == constraint->B_link()->body()->name()) && (contactCheckParam->preState.contacts[j].c1.linkName == constraint->B_link()->name()) && (contactCheckParam->preState.contacts[j].c2.bodyName == constraint->A_link()->body()->name()) && (contactCheckParam->preState.contacts[j].c2.linkName == constraint->A_link()->name()))) skip = true;
        }
        if (!skip && ((ikState==IKState::ATTACH_FIXED) ||
                      (ikState==IKState::DETACH_FIXED))) {
          if (((moveContact.c1.bodyName == constraint->A_link()->body()->name()) && (moveContact.c1.linkName == constraint->A_link()->name()) && (moveContact.c2.bodyName == constraint->B_link()->body()->name()) && (moveContact.c2.linkName == constraint->B_link()->name())) ||
              ((moveContact.c2.bodyName == constraint->A_link()->body()->name()) && (moveContact.c2.linkName == constraint->A_link()->name()) && (moveContact.c1.bodyName == constraint->B_link()->body()->name()) && (moveContact.c1.linkName == constraint->B_link()->name()))) skip = true;
        }
      }
      if (!skip) constraints0.push_back(contactCheckParam->constraints[i]);
    }
    // scfrConstraint
    std::vector<std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> > scfrConstraints;
    for (int b=0; b<contactCheckParam->bodies.size(); b++) {
      if (contactCheckParam->bodies[b]->rootLink()->isFreeJoint())
        {
        std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> scfrConstraint = std::make_shared<ik_constraint2_scfr::ScfrConstraint>();
        scfrConstraint->A_robot() = contactCheckParam->bodies[b];
        scfrConstraints.push_back(scfrConstraint);
      }
    }

    {
      for (int i=0; i<contactCheckParam->preState.contacts.size(); i++) {
        if (contactCheckParam->preState.contacts[i] == moveContact) continue;
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        for (int b=0; b<contactCheckParam->bodies.size(); b++) {
          if (contactCheckParam->bodies[b]->name() != contactCheckParam->preState.contacts[i].c1.bodyName) continue;
          if (contactCheckParam->bodies[b]->link(contactCheckParam->preState.contacts[i].c1.linkName)) {
            constraint->A_link() = contactCheckParam->bodies[b]->link(contactCheckParam->preState.contacts[i].c1.linkName);
            break;
          }
        }
        constraint->A_localpos() = contactCheckParam->preState.contacts[i].c1.localPose;
        for (int b=0; b<contactCheckParam->bodies.size(); b++) {
          if (contactCheckParam->bodies[b]->name() != contactCheckParam->preState.contacts[i].c2.bodyName) continue;
          if (contactCheckParam->bodies[b]->link(contactCheckParam->preState.contacts[i].c2.linkName)) {
            constraint->B_link() = contactCheckParam->bodies[b]->link(contactCheckParam->preState.contacts[i].c2.linkName);
            break;
          }
        }
        constraint->B_localpos() = contactCheckParam->preState.contacts[i].c2.localPose;
        constraint->B_localpos().linear() = constraint->B_localpos().linear() * cnoid::rotFromRpy(0.0, M_PI, M_PI/2).transpose(); // scfrを作る関係上localposのZはrobotの内側を向いている. PositionConstraintで一致させるためにZの向きを揃える.
        constraint->eval_link() = constraint->B_link();
        constraint->eval_localR() = constraint->B_localpos().linear();
        constraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        constraints1.push_back(constraint);
        Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6);
        C.insert(0,2) = 1.0;
        C.insert(1,0) = 1.0; C.insert(1,2) = 0.5;
        C.insert(2,0) = -1.0; C.insert(2,2) = 0.5;
        C.insert(3,1) = 1.0; C.insert(3,2) = 0.5;
        C.insert(4,1) = -1.0; C.insert(4,2) = 0.5;
        C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
        C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
        C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
        C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
        C.insert(9,2) = 0.1; C.insert(9,5) = 1.0;
        C.insert(10,2) = 0.1; C.insert(10,5) = -1.0;
        cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
        cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
        du[0] = 2000.0;
        for (int j=0;j<scfrConstraints.size();j++) {
          // Linkの位置から出す場合、位置固定でも数値誤差によって姿勢が少しずつずれていき、scfr計算の線型計画法に不具合が生じてscfrの領域が潰れる.
          // これを避けるため、staticのときは環境側の接触情報を使う. dynamicのときはbodyごとに2つ以上の接触がありscfrが残ると期待.
          if ((scfrConstraints[j]->A_robot()->name() == contactCheckParam->preState.contacts[i].c1.bodyName) && scfrConstraints[j]->A_robot()->joint(contactCheckParam->preState.contacts[i].c1.linkName)) {
            if (contactCheckParam->preState.contacts[i].c2.isStatic) {
              scfrConstraints[j]->links().push_back(nullptr);
              cnoid::Isometry3 pose = contactCheckParam->preState.contacts[i].c2.localPose;
              for (int b=0; b < contactCheckParam->bodies.size(); b++) {
                if ((contactCheckParam->bodies[b]->name() != contactCheckParam->preState.contacts[i].c2.bodyName) && contactCheckParam->bodies[b]->link(contactCheckParam->preState.contacts[i].c2.linkName)) pose = contactCheckParam->bodies[b]->link(contactCheckParam->preState.contacts[i].c2.linkName)->T() * contactCheckParam->preState.contacts[i].c2.localPose;
              }
              pose.linear() *= cnoid::rotFromRpy(0.0, M_PI, M_PI/2).transpose();
              scfrConstraints[j]->poses().push_back(pose);
            } else {
              scfrConstraints[j]->links().push_back(scfrConstraints[j]->A_robot()->joint(contactCheckParam->preState.contacts[i].c1.linkName));
              scfrConstraints[j]->poses().push_back(contactCheckParam->preState.contacts[i].c1.localPose);
            }
            scfrConstraints[j]->As().emplace_back(0,6);
            scfrConstraints[j]->bs().emplace_back(0);
            scfrConstraints[j]->Cs().push_back(C);
            scfrConstraints[j]->dls().push_back(dl);
            scfrConstraints[j]->dus().push_back(du);
          }
          if ((scfrConstraints[j]->A_robot()->name() == contactCheckParam->preState.contacts[i].c2.bodyName) && scfrConstraints[j]->A_robot()->joint(contactCheckParam->preState.contacts[i].c2.linkName)) {
            if (contactCheckParam->preState.contacts[i].c1.isStatic) {
              scfrConstraints[j]->links().push_back(nullptr);
              cnoid::Isometry3 pose = contactCheckParam->preState.contacts[i].c1.localPose;
              for (int b=0; b < contactCheckParam->bodies.size(); b++) {
                if ((contactCheckParam->bodies[b]->name() != contactCheckParam->preState.contacts[i].c1.bodyName) && contactCheckParam->bodies[b]->link(contactCheckParam->preState.contacts[i].c1.linkName)) pose = contactCheckParam->bodies[b]->link(contactCheckParam->preState.contacts[i].c1.linkName)->T() * contactCheckParam->preState.contacts[i].c1.localPose;
              }
              pose.linear() *= cnoid::rotFromRpy(0.0, M_PI, M_PI/2).transpose();
              scfrConstraints[j]->poses().push_back(pose);
            } else {
              scfrConstraints[j]->links().push_back(scfrConstraints[j]->A_robot()->joint(contactCheckParam->preState.contacts[i].c2.linkName));
              scfrConstraints[j]->poses().push_back(contactCheckParam->preState.contacts[i].c2.localPose);
            }
            scfrConstraints[j]->As().emplace_back(0,6);
            scfrConstraints[j]->bs().emplace_back(0);
            scfrConstraints[j]->Cs().push_back(C);
            scfrConstraints[j]->dls().push_back(dl);
            scfrConstraints[j]->dus().push_back(du);
          }
        }
      }
    }

    std::shared_ptr<ik_constraint2::PositionConstraint> moveContactConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
    for (int b=0; b<contactCheckParam->bodies.size(); b++) {
      if (contactCheckParam->bodies[b]->name() != moveContact.c1.bodyName) continue;
      if (contactCheckParam->bodies[b]->joint(moveContact.c1.linkName)) {
        moveContactConstraint->A_link() = contactCheckParam->bodies[b]->joint(moveContact.c1.linkName);
        break;
      }
    }
    moveContactConstraint->A_localpos() = moveContact.c1.localPose;
    for (int b=0; b<contactCheckParam->bodies.size(); b++) {
      if (contactCheckParam->bodies[b]->name() != moveContact.c2.bodyName) continue;
      if (contactCheckParam->bodies[b]->joint(moveContact.c2.linkName)) {
        moveContactConstraint->B_link() = contactCheckParam->bodies[b]->joint(moveContact.c2.linkName);
        break;
      }
    }
    moveContactConstraint->B_localpos() = moveContact.c2.localPose;
    moveContactConstraint->B_localpos().linear() = moveContactConstraint->B_localpos().linear() * cnoid::rotFromRpy(0.0, M_PI, M_PI/2).transpose(); // scfrを作る関係上localposのZはrobotの内側を向いている. PositionConstraintで一致させるために回転だけ逆にする.

    if ((ikState==IKState::DETACH_FIXED) ||
        (ikState==IKState::ATTACH_PRE)) {
      if (moveContactConstraint->B_link()) moveContactConstraint->B_localpos().translation() += moveContactConstraint->B_link()->R() * moveContactConstraint->B_localpos().linear() * cnoid::Vector3(0,0,0.04);
      else moveContactConstraint->B_localpos().translation() += moveContactConstraint->B_localpos().linear() * cnoid::Vector3(0,0,0.04);
    }
    moveContactConstraint->eval_link() = moveContactConstraint->B_link();
    moveContactConstraint->eval_localR() = moveContactConstraint->B_localpos().linear();
    moveContactConstraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    constraints2.push_back(moveContactConstraint);

    for (int i=0;i<scfrConstraints.size();i++) {
      if (scfrConstraints[i]->poses().size() == 0) return false; // 接触が存在しない物体がある.
      if (!ik_constraint2_keep_collision_scfr::checkSCFRExistance(scfrConstraints[i]->poses(), scfrConstraints[i]->As(), scfrConstraints[i]->bs(), scfrConstraints[i]->Cs(), scfrConstraints[i]->dls(), scfrConstraints[i]->dus(), scfrConstraints[i]->A_robot()->mass(), scfrConstraints[i]->SCFRParam())) return false; // scfrが消えている
      constraints0.push_back(scfrConstraints[i]);
    }

    nominals.insert(nominals.end(), contactCheckParam->nominals.begin(), contactCheckParam->nominals.end());
    double goalPrecision = std::static_pointer_cast<ik_constraint2::PositionConstraint>(contactCheckParam->goalConstraint)->precision();
    std::static_pointer_cast<ik_constraint2::PositionConstraint>(contactCheckParam->goalConstraint)->precision() = 1e10;
    nominals.push_back(contactCheckParam->goalConstraint);

    bool solved = false;
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraint{constraints0, constraints1, constraints2, nominals};

    std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks;
    solved  =  prioritized_inverse_kinematics_solver2::solveIKLoop(contactCheckParam->variables,
                                                                   constraint,
                                                                   contactCheckParam->rejections,
                                                                   prevTasks,
                                                                   contactCheckParam->pikParam,
                                                                   tmpPath
                                                                   );
    if(!solved) {
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > gikConstraints{constraints0, constraints1};
      global_inverse_kinematics_solver::GIKParam gikParam = contactCheckParam->gikParam;
      gikParam.projectLink.resize(1);
      gikParam.projectLink[0] = moveContactConstraint->A_link() ? moveContactConstraint->A_link() : moveContactConstraint->B_link();
      gikParam.projectLocalPose = moveContactConstraint->A_link() ? moveContactConstraint->A_localpos() : moveContactConstraint->B_localpos();
      // 関節角度上下限を厳密に満たしていないと、omplのstart stateがエラーになるので
      for(int i=0;i<contactCheckParam->variables.size();i++){
        if(contactCheckParam->variables[i]->isRevoluteJoint() || contactCheckParam->variables[i]->isPrismaticJoint()) {
          contactCheckParam->variables[i]->q() = std::max(std::min(contactCheckParam->variables[i]->q(),contactCheckParam->variables[i]->q_upper()),contactCheckParam->variables[i]->q_lower());
        }
      }
      solved = global_inverse_kinematics_solver::solveGIK(contactCheckParam->variables,
                                                          gikConstraints,
                                                          constraints2,
                                                          nominals,
                                                          gikParam,
                                                          tmpPath);
    }

    std::static_pointer_cast<ik_constraint2::PositionConstraint>(contactCheckParam->goalConstraint)->precision() = goalPrecision;
    if (solved && ((ikState==IKState::ATTACH_FIXED) || (ikState==IKState::DETACH_FIXED))) {
      postState.transition.insert(postState.transition.end(), (*tmpPath).begin(), (*tmpPath).end());
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraint_goal;
      constraint_goal.push_back(constraints0);
      constraint_goal.push_back(constraints1);
      if (ikState==IKState::ATTACH_FIXED) constraint_goal.push_back(constraints2);
      constraint_goal.push_back(std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >{contactCheckParam->goalConstraint});
      constraint_goal.push_back(contactCheckParam->nominals);
      std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks_;
      prioritized_inverse_kinematics_solver2::solveIKLoop(contactCheckParam->variables,
                                                          constraint_goal,
                                                          contactCheckParam->rejections,
                                                          prevTasks_,
                                                          contactCheckParam->pikParam,
                                                          tmpPath
                                                          );
    }

    // for ( int i=0; i<constraints0.size(); i++ ) {
    //   std::cerr << "constraints0: "<< constraints0[i]->isSatisfied() << std::endl;
    // }
    // for ( int i=0; i<constraints1.size(); i++ ) {
    //   std::cerr << "constraints1: "<< constraints1[i]->isSatisfied() << std::endl;
    // }
    // for ( int i=0; i<constraints2.size(); i++ ) {
    //   constraints2[i]->debugLevel() = 2;
    //   constraints2[i]->updateBounds();
    //   std::cerr << "constraints2: "<< constraints2[i]->isSatisfied() << std::endl;
    //   constraints2[i]->debugLevel() = 0;
    // }
    // for ( int i=0; i<nominals.size(); i++ ) {
    //   std::cerr << "nominals: "<< nominals[i]->isSatisfied() << std::endl;
    // }

    if (solved) {
      postState.transition.insert(postState.transition.end(), (*tmpPath).begin(), (*tmpPath).end());
    }

    return solved;

  }

  void WholeBodyContactPlanner::goalPath(std::vector<ContactState>& path) {
    if (!this->goal()) {
      std::cerr << "[WholeBodyContactPlanner] goal not found!!" << std::endl;
    } else {
      path.clear();
      path.push_back(std::static_pointer_cast<contact_before_motion::ContactNode>(this->goal())->state());
      std::weak_ptr<graph_search::Node> node = this->goal()->parent();
      while (!node.expired()) {
        path.push_back(std::static_pointer_cast<contact_before_motion::ContactNode>(node.lock())->state());
        node = node.lock()->parent();
      }
    }
    std::reverse(path.begin(), path.end());
  }

}
