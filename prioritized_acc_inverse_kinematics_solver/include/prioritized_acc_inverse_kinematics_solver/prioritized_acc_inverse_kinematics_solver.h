#ifndef PRIORITIZED_ACC_INVERSE_KINEMATICS_SOLVER_PRIORITIZED_ACC_INVERSE_KINEMATICS_SOLVER_H
#define PRIORITIZED_ACC_INVERSE_KINEMATICS_SOLVER_PRIORITIZED_ACC_INVERSE_KINEMATICS_SOLVER_H

#include <cnoid/Body>
#include <aik_constraint/IKConstraint.h>
#include <prioritized_qp_base/PrioritizedQPBaseSolver.h>
#include <prioritized_qp_osqp/prioritized_qp_osqp.h>

namespace prioritized_acc_inverse_kinematics_solver {
  /*
    事前に、robot::calcForwardKinematics(True,True)とrobot::calcCenterOfMass()を行い、全リンクのF_extは0にしてからcalcInverseDynamics()してrootLinkが受ける力(rootLinkまわり)をrootLink->F_ext()に入れること.
    variables: 動かして良いjoint (free jointは6DOF扱い)
    forces: 発生する接触力.
    ikc_list: タスクたち. vectorの前の要素の方が高優先度. 0番目の要素は必ず満たすと仮定しQPを解かない
    prevTasks: 前回のtasksを入れる. 自動的に更新される.
   */
  class IKParam {
  public:
    double ddqWeight = 1e-6;
    std::vector<double> ddqWeightVec; // ddqWeightVec.size() == dim(variables)の場合、探索変数の各要素について、wnをddqWeightVec倍する. 通常はddqWeight倍する.
    double forceWeight = 1e-12;
    std::vector<double> forceWeightVec; // forceWeightVec.size() == dim(forces)の場合、探索変数の各要素について、wnをforceWeightVec倍する. 通常はforceWeight倍する.
    double wn = 1e0;
    std::vector<double> wnVec; // wnVec.size() == ikc_list.size()の場合、wnの代わりにこっちを使う
    int debugLevel = 0;
  };
  bool solveAIK (const std::vector<cnoid::LinkPtr>& variables,
                 const std::vector<std::shared_ptr<aik_constraint::Force> >& forces,
                 const std::vector<std::vector<std::shared_ptr<aik_constraint::IKConstraint> > >& ikc_list,
                 std::vector<std::shared_ptr<prioritized_qp_base::Task> >& prevTasks,
                 const IKParam& param = IKParam(),
                 std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)> taskGeneratorFunc = [](std::shared_ptr<prioritized_qp_base::Task>& task, int debugLevel){
                   std::shared_ptr<prioritized_qp_osqp::Task> taskOSQP = std::dynamic_pointer_cast<prioritized_qp_osqp::Task>(task);
                   if(!taskOSQP){
                     task = std::make_shared<prioritized_qp_osqp::Task>();
                     taskOSQP = std::dynamic_pointer_cast<prioritized_qp_osqp::Task>(task);
                   }
                   taskOSQP->settings().verbose = (debugLevel>=2);
                   taskOSQP->settings().max_iter = 4000;
                   taskOSQP->settings().eps_abs = 1e-3;// 大きい方が速いが，不正確. 1e-5はかなり小さい. 1e-4は普通
                   taskOSQP->settings().eps_rel = 1e-3;// 大きい方が速いが，不正確. 1e-5はかなり小さい. 1e-4は普通
                   taskOSQP->settings().scaled_termination = true;// avoid too severe termination check
                 }
                 );

}

#endif
