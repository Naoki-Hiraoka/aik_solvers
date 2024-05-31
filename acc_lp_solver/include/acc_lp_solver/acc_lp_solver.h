#ifndef PRIORITIZED_ACC_INVERSE_KINEMATICS_SOLVER_H
#define PRIORITIZED_ACC_INVERSE_KINEMATICS_SOLVER_H

#include <cnoid/Body>
#include <aik_constraint/IKConstraint.h>

namespace acc_lp_solver {
  /*
    事前に、robot::calcForwardKinematics(True,True)とrobot::calcCenterOfMass()を行い、全リンクのF_extは0にしてからcalcInverseDynamics()してrootLinkが受ける力(rootLinkまわり)をrootLink->F_ext()に入れること.
    variables: 動かして良いjoint (free jointは6DOF扱い)
    forces: 発生する接触力.
    ikc_list: タスクたち.
    force: この値を最大化する. forcesに含めよ
   */
  class IKParam {
  public:
    int debugLevel = 0;
    double maxValue = 1e5; // 接触力を表す変数の最大・最小値. 制約A,Cがゆるい場合、無限に力を発揮可能なためLPの解が定まらないことから、maxValueで制限する.
    double lpTolerance = 1e-7;//default 1e-7. vertexを見逃さないようにする. 1e-12などにする
  };
  bool solveLP (const std::shared_ptr<aik_constraint::Force>& force,
                const std::vector<cnoid::LinkPtr>& variables,
                const std::vector<std::shared_ptr<aik_constraint::Force> >& forces,
                const std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& ikc_list,
                const IKParam& param = IKParam()
                );

}

#endif
