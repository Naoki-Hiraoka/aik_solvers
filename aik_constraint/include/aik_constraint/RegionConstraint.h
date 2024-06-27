#ifndef AIK_CONSTRAINT_REGIONCONSTRAINT_H
#define AIK_CONSTRAINT_REGIONCONSTRAINT_H

#include <aik_constraint/IKConstraint.h>
#include <cnoid/EigenUtil>
#include <cnoid/LinkPath>
#include <iostream>

namespace aik_constraint{
  class RegionConstraint : public IKConstraint
  {
  public:
    //A_link中のA_localposの部位とB_link中のB_localposの部位を一致させる.
    //  このとき、eval_R系で見たA-Bの位置エラーが、region内にあるようにする.
    //  ref_acc: eval_frame. feedforward目標加速度(A-B). C * (acc - ref_acc + pgain * error + dgain * derror)がdl,du内になるように目標加速度accを決める
    //  maxAcc: 目標加速度の頭打ち eval座標系. 目標加速度をmaxAccで頭打ちしてからweight倍したものがgetIneq()で返る
    //  weight: コスト関数の重み. error * weight^2 * error. eval座標系
    //  link: parent link. nullptrならworld座標系を意味する
    //  localpos: parent link frame
    //  localvel: parent link frame. endeffector origin
    const cnoid::LinkPtr& A_link() const { return A_link_;}
    cnoid::LinkPtr& A_link() { return A_link_;}
    const cnoid::Vector3& A_localpos() const { return A_localpos_;}
    cnoid::Vector3& A_localpos() { return A_localpos_;}
    const cnoid::Vector3& A_localvel() const { return A_localvel_;}
    cnoid::Vector3& A_localvel() { return A_localvel_;}
    const cnoid::LinkPtr& B_link() const { return B_link_;}
    cnoid::LinkPtr& B_link() { return B_link_;}
    const cnoid::Vector3& B_localpos() const { return B_localpos_;}
    cnoid::Vector3& B_localpos() { return B_localpos_;}
    const cnoid::Vector3& B_localvel() const { return B_localvel_;}
    cnoid::Vector3& B_localvel() { return B_localvel_;}
    const cnoid::Vector3& ref_acc() const { return ref_acc_;}
    cnoid::Vector3& ref_acc() { return ref_acc_;}
    const double& pgain() const { return pgain_;}
    double& pgain() { return pgain_;}
    const double& dgain() const { return dgain_;}
    double& dgain() { return dgain_;}
    const double& maxAcc() const { return maxAcc_;}
    double& maxAcc() { return maxAcc_;}
    const double& weight() const { return weight_;}
    double& weight() { return weight_;}
    const cnoid::LinkPtr& eval_link() const { return eval_link_;}
    cnoid::LinkPtr& eval_link() { return eval_link_;}
    const cnoid::Matrix3d& eval_localR() const { return eval_localR_;}
    cnoid::Matrix3d& eval_localR() { return eval_localR_;}

    const Eigen::SparseMatrix<double,Eigen::RowMajor>& C() const { return C_;}
    Eigen::SparseMatrix<double,Eigen::RowMajor>& C() { return C_;}
    const cnoid::VectorX& dl() const { return dl_;}
    cnoid::VectorX& dl() { return dl_;}
    const cnoid::VectorX& du() const { return du_;}
    cnoid::VectorX& du() { return du_;}

    // 内部状態更新
    void update (const std::vector<cnoid::LinkPtr>& joints, const std::vector<std::shared_ptr<Force> >& forces) override;

    // for debug view
    const std::vector<cnoid::SgNodePtr>& getDrawOnObjects() override;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    cnoid::LinkPtr A_link_ = nullptr;
    cnoid::Vector3 A_localpos_ = cnoid::Vector3::Zero();
    cnoid::Vector3 A_localvel_ = cnoid::Vector3::Zero();
    cnoid::LinkPtr B_link_ = nullptr;
    cnoid::Vector3 B_localpos_ = cnoid::Vector3::Zero();
    cnoid::Vector3 B_localvel_ = cnoid::Vector3::Zero();
    cnoid::Vector3 ref_acc_ = cnoid::Vector3::Zero();
    double pgain_ = 400;
    double dgain_ = 50;
    double maxAcc_ = 15;
    double weight_ = 1.0;
    cnoid::LinkPtr eval_link_ = nullptr;
    cnoid::Matrix3d eval_localR_ = cnoid::Matrix3d::Identity();
    Eigen::SparseMatrix<double,Eigen::RowMajor> C_{0,3}; // ? x 3
    Eigen::VectorXd dl_;
    Eigen::VectorXd du_;

    cnoid::SgLineSetPtr lines_;

    cnoid::LinkPtr jacobian_A_link_ = nullptr;// 前回のjacobian計算時のA_link
    cnoid::LinkPtr jacobian_B_link_ = nullptr;// 前回のjacobian計算時のB_link
    std::vector<cnoid::LinkPtr> jacobian_joints_; // 前回のjacobian計算時のjoints
    std::vector<std::shared_ptr<Force> > jacobian_forces_; // 前回のjacobian計算時のforces

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
