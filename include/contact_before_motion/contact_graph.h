#pragma once

#include <graph_search/graph.h>
#include <contact_before_motion/util.h>
#include <contact_before_motion/contact_state.h>
#include <contact_before_motion/contact_node.h>
#include <ik_constraint2_distance_field/ik_constraint2_distance_field.h>
#include <ik_constraint2_bullet/ik_constraint2_bullet.h>
#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>

namespace contact_before_motion{
  class WholeBodyContactPlanner : public graph_search::Planner {
  public:
    std::vector<cnoid::BodyPtr> bodies;
    std::vector<cnoid::LinkPtr> variables;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rejections;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
    std::shared_ptr<ik_constraint2::IKConstraint> goalConstraint;
    std::shared_ptr<ContactState> currentContactState;
    std::vector<std::shared_ptr<ContactCandidate> > contactDynamicCandidates;
    std::vector<std::shared_ptr<ContactCandidate> > contactStaticCandidates; // staticCondidate同士の接触は起こりえない
    prioritized_inverse_kinematics_solver2::IKParam pikParam;
    global_inverse_kinematics_solver::GIKParam gikParam;
    std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField> field;
    double addCandidateDistance = 2.0; // contactDynamicCandidateのルートリンクからこの距離を超えるものはgikを使うまでもなく解けないものとする

    WholeBodyContactPlanner() {
      gikParam.maxTranslation = 2.0;
      gikParam.pikParam.maxIteration = 100; // max iterationに達するか、convergeしたら終了する. isSatisfiedでは終了しない. ゼロ空間でreference angleに可能な限り近づけるタスクがあるので. 1 iterationで0.5msくらいかかるので、stateを1つ作るための時間の上限が見積もれる. 一見、この値を小さくすると早くなりそうだが、goalSampling時に本当はgoalに到達できるのにその前に返ってしまうことで遅くなることがあるため、少ないiterationでも収束するように他のパラメータを調整したほうがいい
      gikParam.pikParam.minIteration = 20;
      gikParam.pikParam.checkFinalState = true;
      gikParam.pikParam.calcVelocity = false;
      gikParam.delta = 0.01; // この距離内のstateは、中間のconstraintチェック無しで遷移可能. stateごとの距離がこの距離以内だとそもそも同じstateとみなされてあたらしくstateを作らない. 足を浮かせるとき等はstateが大きく変化しないので、deltaも小さくしておかないとstateが増えない.
      gikParam.projectCellSize = 0.02;
      gikParam.threads = 1;
      gikParam.timeout = 1.0;
      gikParam.goalBias = 0.2;
      gikParam.pikParam.we = 1e1; // 逆運動学が振動しないこと優先. 1e0だと不安定. 1e3だと大きすぎる
      gikParam.pikParam.wmax = 1e0; // 1e2程度にすると関節がめり込まなくなるが、ほとんど動かない.
      gikParam.pikParam.convergeThre = 5e-3;
      gikParam.modelMutex = std::make_shared<std::mutex>();

      pikParam.checkFinalState=true;
      pikParam.calcVelocity = false;
      pikParam.debugLevel = 0;
      pikParam.we = 1e2;
      pikParam.wmax = 1e1;
      pikParam.convergeThre = 5e-3;
      pikParam.maxIteration = 100;
    }

  public:
    class ContactTransitionCheckParam : public graph_search::Planner::TransitionCheckParam {
    public:
      ContactState preState;
      ContactState postState;
      std::vector<cnoid::BodyPtr> bodies;
      std::vector<cnoid::LinkPtr> variables;
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints;
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rejections;
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
      std::shared_ptr<ik_constraint2::IKConstraint> goalConstraint;
      std::vector<std::shared_ptr<ContactCandidate> > contactDynamicCandidates;
      std::vector<std::shared_ptr<ContactCandidate> > contactStaticCandidates;
      prioritized_inverse_kinematics_solver2::IKParam pikParam;
      global_inverse_kinematics_solver::GIKParam gikParam;
      int debugLevel = 0;
      double addCandidateDistance = 2.0;
    };
    virtual std::shared_ptr<graph_search::Planner::TransitionCheckParam> generateCheckParam() override;
    void preCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> extend_node) override;
    void postCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> extend_node) override;
    bool checkTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override;
    bool isGoalSatisfied(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override;
    void calcHeuristic(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> node) override;
    std::vector<std::shared_ptr<graph_search::Node> > gatherAdjacentNodes(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override;
    void addNodes2Graph(std::vector<std::shared_ptr<graph_search::Node> >& nodes) override;
    void cloneCheckParam(std::shared_ptr<ContactTransitionCheckParam> checkParam);
    bool checkTransitionImpl(std::shared_ptr<const ContactTransitionCheckParam> checkParam,
                             ContactState& postState
                             ); // preStateからpostStateまでの遷移が可能ならtrue. trueのとき、postStateのframeやlocalPoseを書き換える.
    bool solveContactIK(std::shared_ptr<const ContactTransitionCheckParam> checkParam,
                        Contact& moveContact,
                        ContactState& postState,
                        const IKState& ikState
                        );
    bool solve();
    void goalPath(std::vector<ContactState>& path);

  };
}
