#ifndef AIK_CONSTRAINT_COMCONSTRAINT_H
#define AIK_CONSTRAINT_COMCONSTRAINT_H

#include <aik_constraint/IKConstraint.h>
#include <cnoid/EigenUtil>
#include <cnoid/SceneMarkers>
#include <iostream>

namespace aik_constraint{
  class COMConstraint : public IKConstraint
  {
  public:
    //robotの重心をworld座標系のtargetPosに位置させる.
    //  maxError: エラーの頭打ち
    //  precision: 収束判定の閾値
    //  weight: コスト関数の重み. error * weight^2 * error. 0の成分はjacobianやerrorに含まれない
    //状態が更新される度に, 手動でcalcForwardKinematics()とcalcCenterOfMass()を呼ぶ必要が有る.
    const cnoid::BodyPtr& A_robot() const { return A_robot_;}
    cnoid::BodyPtr& A_robot() { return A_robot_;}
    const cnoid::Vector3& A_localp() const { return A_localp_;}
    cnoid::Vector3& A_localp() { return A_localp_;}
    const cnoid::Vector3& A_localv() const { return A_localv_;}
    cnoid::Vector3& A_localv() { return A_localv_;}
    const cnoid::BodyPtr& B_robot() const { return B_robot_;}
    cnoid::BodyPtr& B_robot() { return B_robot_;}
    const cnoid::Vector3& B_localp() const { return B_localp_;}
    cnoid::Vector3& B_localp() { return B_localp_;}
    const cnoid::Vector3& B_localv() const { return B_localv_;}
    cnoid::Vector3& B_localv() { return B_localv_;}
    const cnoid::Matrix3d& eval_R() const { return eval_R_;}
    cnoid::Matrix3d& eval_R() { return eval_R_;}

    const cnoid::Vector3& ref_acc() const { return ref_acc_;}
    cnoid::Vector3& ref_acc() { return ref_acc_;}
    const cnoid::Vector3& pgain() const { return pgain_;}
    cnoid::Vector3& pgain() { return pgain_;}
    const cnoid::Vector3& dgain() const { return dgain_;}
    cnoid::Vector3& dgain() { return dgain_;}
    const cnoid::Vector3& maxAcc() const { return maxAcc_;}
    cnoid::Vector3& maxAcc() { return maxAcc_;}
    const cnoid::Vector3& maxAccByPosError() const { return maxAccByPosError_;}
    cnoid::Vector3& maxAccByPosError() { return maxAccByPosError_;}
    const cnoid::Vector3& maxAccByVelError() const { return maxAccByVelError_;}
    cnoid::Vector3& maxAccByVelError() { return maxAccByVelError_;}
    const cnoid::Vector3& weight() const { return weight_;}
    cnoid::Vector3& weight() { return weight_;}

    void update (const std::vector<cnoid::LinkPtr>& joints) override;

    // for debug view
    const std::vector<cnoid::SgNodePtr>& getDrawOnObjects() override;

  protected:
    cnoid::BodyPtr A_robot_ = nullptr;
    cnoid::Vector3 A_localp_ = cnoid::Vector3::Zero();
    cnoid::Vector3 A_localv_ = cnoid::Vector3::Zero();
    cnoid::BodyPtr B_robot_ = nullptr;
    cnoid::Vector3 B_localp_ = cnoid::Vector3::Zero();
    cnoid::Vector3 B_localv_ = cnoid::Vector3::Zero();
    cnoid::Matrix3d eval_R_ = cnoid::Matrix3d::Identity();

    cnoid::Vector3 ref_acc_ = cnoid::Vector3::Zero();
    cnoid::Vector3 pgain_ = 400 * cnoid::Vector3::Ones();
    cnoid::Vector3 dgain_ = 50 * cnoid::Vector3::Ones();
    cnoid::Vector3 maxAcc_ = 15 * cnoid::Vector3::Ones(); // 歩行では10は出る. 15もたまに出る
    cnoid::Vector3 maxAccByPosError_ = 5 * cnoid::Vector3::Ones();
    cnoid::Vector3 maxAccByVelError_ = 10 * cnoid::Vector3::Ones();
    cnoid::Vector3 weight_ = cnoid::Vector3::Ones();

    cnoid::SgLineSetPtr lines_;

    cnoid::BodyPtr jacobian_A_robot_ = nullptr;// 前回のjacobian計算時のrobot
    cnoid::BodyPtr jacobian_B_robot_ = nullptr;// 前回のjacobian計算時のrobot
    std::vector<cnoid::LinkPtr> jacobian_joints_; // 前回のjacobian計算時のjoints
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_full_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_full_local_;
    std::unordered_map<cnoid::LinkPtr,int> jacobianColMap_;

  };
}

#endif
