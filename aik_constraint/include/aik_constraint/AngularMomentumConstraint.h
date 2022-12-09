#ifndef AIK_CONSTRAINT_ANGULARMOMENTUMCONSTRAINT_H
#define AIK_CONSTRAINT_ANGULARMOMENTUMCONSTRAINT_H

#include <aik_constraint/IKConstraint.h>
#include <cnoid/EigenUtil>
#include <cnoid/SceneMarkers>
#include <iostream>

namespace aik_constraint{
  class AngularMomentumConstraint : public IKConstraint
  {
  public:
    //robotの重心周りの角運動量を目標の値[kg m^2/s]に一致させる
    //  内部の処理では角運動量を重心周りのイナーシャで割って、[rad/s]の次元で扱う
    //  dt: [s]
    //  maxError: エラーの頭打ち[rad]
    //  weight: コスト関数の重み. error * weight^2 * error.
    //  targetAngularMomentum: 重心周り. ワールド座標系. [kg m^2/s]
    //状態が更新される度に, 手動でcalcForwardKinematics()とcalcCenterOfMass()を呼ぶ必要が有る.
    const cnoid::BodyPtr& robot() const { return robot_;}
    cnoid::BodyPtr& robot() { return robot_;}
    const cnoid::Matrix3d& eval_R() const { return eval_R_;}
    cnoid::Matrix3d& eval_R() { return eval_R_;}

    const cnoid::Vector3& target_Iw() const { return target_Iw_;}
    cnoid::Vector3& target_Iw() { return target_Iw_;}
    const cnoid::Vector3& ref_Idw() const { return ref_Idw_;}
    cnoid::Vector3& ref_Idw() { return ref_Idw_;}

    const cnoid::Vector3& dgain() const { return dgain_;}
    cnoid::Vector3& dgain() { return dgain_;}

    const cnoid::Vector3& maxAcc() const { return maxAcc_;}
    cnoid::Vector3& maxAcc() { return maxAcc_;}
    const cnoid::Vector3& maxAccByVelError() const { return maxAccByVelError_;}
    cnoid::Vector3& maxAccByVelError() { return maxAccByVelError_;}

    const cnoid::Vector3& weight() const { return weight_;}
    cnoid::Vector3& weight() { return weight_;}

    void update (const std::vector<cnoid::LinkPtr>& joints) override;

  protected:
    cnoid::BodyPtr robot_ = nullptr;
    cnoid::Matrix3d eval_R_ = cnoid::Matrix3d::Identity();

    cnoid::Vector3 target_Iw_ = cnoid::Vector3::Zero();
    cnoid::Vector3 ref_Idw_ = cnoid::Vector3::Zero();

    cnoid::Vector3 dgain_ = 50 * cnoid::Vector3::Ones();
    cnoid::Vector3 maxAcc_ = 15 * cnoid::Vector3::Ones(); // 歩行では10は出る. 15もたまに出る
    cnoid::Vector3 maxAccByVelError_ = 10 * cnoid::Vector3::Ones();
    cnoid::Vector3 weight_ = cnoid::Vector3::Ones();

    cnoid::BodyPtr jacobian_robot_ = nullptr;// 前回のjacobian計算時のrobot
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_full_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_full_local_;
    std::vector<cnoid::LinkPtr> jacobian_joints_; // 前回のjacobian計算時のjoints
    std::unordered_map<cnoid::LinkPtr,int> jacobianColMap_;
  };

}

#endif
