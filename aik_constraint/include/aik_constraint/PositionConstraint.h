#ifndef AIK_CONSTRAINT_POSITIONCONSTRAINT_H
#define AIK_CONSTRAINT_POSITIONCONSTRAINT_H

#include <aik_constraint/IKConstraint.h>
#include <cnoid/EigenUtil>
#include <cnoid/LinkPath>
#include <iostream>

namespace aik_constraint{
  class PositionConstraint : public IKConstraint
  {
  public:
    //A_link中のA_localposの部位とB_link中のB_localposの部位を一致させる.
    //  pgain: evel座標系
    //  dgain: evel座標系
    //  ref_acc: eval_frame. feedforward目標加速度(B-A). ref_acc + pgain * error + dgain * derrorが目標加速度になる
    //  maxAcc: 目標加速度の頭打ち eval座標系. 目標加速度をmaxAccで頭打ちしてからweight倍したものがgetEq()で返る
    //  maxAccByPosError: 目標加速度の頭打ち eval座標系.
    //  maxAccByVelError: 目標加速度の頭打ち eval座標系.
    //  weight: コスト関数の重み. error * weight^2 * error. 0の成分はjacobianやerrorに含まれない. eval座標系
    //  link: parent link. nullptrならworld座標系を意味する
    //  localpos: parent link frame
    //  localvel: parent link frame. endeffector origin
    const cnoid::LinkPtr& A_link() const { return A_link_;}
    cnoid::LinkPtr& A_link() { return A_link_;}
    const cnoid::Position& A_localpos() const { return A_localpos_;}
    cnoid::Position& A_localpos() { return A_localpos_;}
    const cnoid::Vector6& A_localvel() const { return A_localvel_;}
    cnoid::Vector6& A_localvel() { return A_localvel_;}
    const cnoid::LinkPtr& B_link() const { return B_link_;}
    cnoid::LinkPtr& B_link() { return B_link_;}
    const cnoid::Position& B_localpos() const { return B_localpos_;}
    cnoid::Position& B_localpos() { return B_localpos_;}
    const cnoid::Vector6& B_localvel() const { return B_localvel_;}
    cnoid::Vector6& B_localvel() { return B_localvel_;}
    const cnoid::Vector6& ref_acc() const { return ref_acc_;}
    cnoid::Vector6& ref_acc() { return ref_acc_;}
    const cnoid::Vector6& pgain() const { return pgain_;}
    cnoid::Vector6& pgain() { return pgain_;}
    const cnoid::Vector6& dgain() const { return dgain_;}
    cnoid::Vector6& dgain() { return dgain_;}
    const cnoid::Vector6& maxAcc() const { return maxAcc_;}
    cnoid::Vector6& maxAcc() { return maxAcc_;}
    const cnoid::Vector6& maxAccByPosError() const { return maxAccByPosError_;}
    cnoid::Vector6& maxAccByPosError() { return maxAccByPosError_;}
    const cnoid::Vector6& maxAccByVelError() const { return maxAccByVelError_;}
    cnoid::Vector6& maxAccByVelError() { return maxAccByVelError_;}
    const cnoid::Vector6& weight() const { return weight_;}
    cnoid::Vector6& weight() { return weight_;}
    const cnoid::LinkPtr& eval_link() const { return eval_link_;}
    cnoid::LinkPtr& eval_link() { return eval_link_;}
    const cnoid::Matrix3d& eval_localR() const { return eval_localR_;}
    cnoid::Matrix3d& eval_localR() { return eval_localR_;}

    // 内部状態更新
    void update (const std::vector<cnoid::LinkPtr>& joints) override;

    // for debug view
    const std::vector<cnoid::SgNodePtr>& getDrawOnObjects() override;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    cnoid::LinkPtr A_link_ = nullptr;
    cnoid::Position A_localpos_ = cnoid::Position::Identity();
    cnoid::Vector6 A_localvel_ = cnoid::Vector6::Zero();
    cnoid::LinkPtr B_link_ = nullptr;
    cnoid::Position B_localpos_ = cnoid::Position::Identity();
    cnoid::Vector6 B_localvel_ = cnoid::Vector6::Zero();
    cnoid::Vector6 ref_acc_ = cnoid::Vector6::Zero();
    cnoid::Vector6 pgain_ = 400 * cnoid::Vector6::Ones();
    cnoid::Vector6 dgain_ = 50 * cnoid::Vector6::Ones();
    cnoid::Vector6 maxAcc_ = 15 * cnoid::Vector6::Ones(); // 歩行では10は出る. 15もたまに出る
    cnoid::Vector6 maxAccByPosError_ = 5 * cnoid::Vector6::Ones();
    cnoid::Vector6 maxAccByVelError_ = 10 * cnoid::Vector6::Ones();
    cnoid::Vector6 weight_ = cnoid::Vector6::Ones();
    cnoid::LinkPtr eval_link_ = nullptr;
    cnoid::Matrix3d eval_localR_ = cnoid::Matrix3d::Identity();

    cnoid::SgLineSetPtr lines_;

    cnoid::LinkPtr jacobian_A_link_ = nullptr;// 前回のjacobian計算時のA_link
    cnoid::LinkPtr jacobian_B_link_ = nullptr;// 前回のjacobian計算時のB_link
    std::vector<cnoid::LinkPtr> jacobian_joints_; // 前回のjacobian計算時のjoints

    std::vector<cnoid::LinkPtr> path_A_joints_;
    std::vector<cnoid::LinkPtr> path_B_joints_;
    std::vector<cnoid::LinkPtr> path_BA_joints_;
    int path_BA_joints_numUpwardConnections_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_full_;
    std::unordered_map<cnoid::LinkPtr,int> jacobianColMap_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_full_local_;
  };
}

#endif
