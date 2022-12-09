#ifndef AIK_CONSTRAINT_JOINTLIMITCONSTRAINT_H
#define AIK_CONSTRAINT_JOINTLIMITCONSTRAINT_H

#include <aik_constraint/IKConstraint.h>
#include <cnoid/EigenUtil>

namespace aik_constraint{
  class JointLimitConstraint : public IKConstraint
  {
  public:
    //jointのqをq_upperとq_lowerの間にさせる.
    //  maxError: エラーの頭打ち
    //  precision: 収束判定の閾値
    //  weight: コスト関数の重み. error * weight^2 * error. maxErrorの適用後に適用する

    const cnoid::LinkPtr& joint() const { return joint_;}
    cnoid::LinkPtr& joint() { return joint_;}
    const double& pgain() const { return pgain_;}
    double& pgain() { return pgain_;}
    const double& dgain() const { return dgain_;}
    double& dgain() { return dgain_;}
    const double& maxAcc() const { return maxAcc_;}
    double& maxAcc() { return maxAcc_;}
    const double& maxAccByPosError() const { return maxAccByPosError_;}
    double& maxAccByPosError() { return maxAccByPosError_;}
    const double& maxAccByVelError() const { return maxAccByVelError_;}
    double& maxAccByVelError() { return maxAccByVelError_;}
    const double& weight() const { return weight_;}
    double& weight() { return weight_;}

    void update (const std::vector<cnoid::LinkPtr>& joints) override;

  protected:
    virtual double get_q_lower() { return this->joint_->q_lower(); } // this->joint_はnullptrではない前提
    virtual double get_q_upper() { return this->joint_->q_upper(); } // this->joint_はnullptrではない前提

    cnoid::LinkPtr joint_ = nullptr;
    double pgain_ = 400;
    double dgain_ = 50;
    double maxAcc_ = 15;
    double maxAccByPosError_ = 5;
    double maxAccByVelError_ = 10;
    double weight_ = 1.0;

    cnoid::LinkPtr jacobianIneq_joint_ = nullptr; //前回jacobian_を計算した時のjoint
    std::vector<cnoid::LinkPtr> jacobianIneq_joints_; // 前回のjacobian計算時のjoints
    std::unordered_map<cnoid::LinkPtr,int> jacobianIneqColMap_;
  };
}

#endif
