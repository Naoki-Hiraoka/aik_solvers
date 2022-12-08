#ifndef AIKCONSTRAINT_JOINTANGLECONSTRAINT_H
#define AIKCONSTRAINT_JOINTANGLECONSTRAINT_H

#include <aik_constraint/IKConstraint.h>
#include <cnoid/EigenUtil>

namespace aik_constraint{
  class JointAngleConstraint : public IKConstraint
  {
  public:
    //jointのqとtargetqを一致させる.
    //  maxError: エラーの頭打ち
    //  precision: 収束判定の閾値
    //  weight: コスト関数の重み. error * weight^2 * error.

    const cnoid::LinkPtr& joint() const { return joint_;}
    cnoid::LinkPtr& joint() { return joint_;}
    const double& targetq() const { return targetq_;}
    double& targetq() { return targetq_;}
    const double& targetdq() const { return targetdq_;}
    double& targetdq() { return targetdq_;}
    const double& ref_acc() const { return ref_acc_;}
    double& ref_acc() { return ref_acc_;}
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

  private:
    cnoid::LinkPtr joint_ = nullptr;
    double targetq_ = 0.0;
    double targetdq_ = 0.0;
    double ref_acc_ = 0.0;
    double pgain_ = 400;
    double dgain_ = 50;
    double maxAcc_ = 15;
    double maxAccByPosError_ = 5;
    double maxAccByVelError_ = 10;
    double weight_ = 1.0;

    cnoid::LinkPtr jacobian_joint_ = nullptr; //前回jacobian_を計算した時のjoint
    std::vector<cnoid::LinkPtr> jacobian_joints_; // 前回のjacobian計算時のjoints
    std::unordered_map<cnoid::LinkPtr,int> jacobianColMap_;
  };
}

#endif
