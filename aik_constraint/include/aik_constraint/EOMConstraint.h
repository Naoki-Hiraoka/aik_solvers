#ifndef AIK_CONSTRAINT_EOMCONSTRAINT_H
#define AIK_CONSTRAINT_EOMCONSTRAINT_H

#include <aik_constraint/IKConstraint.h>
#include <cnoid/EigenUtil>
#include <iostream>

namespace aik_constraint{
  class EOMConstraint : public IKConstraint
  {
  public:
    // robotの力の釣り合いの制約. world系. 重心周り. (6次元)
    //  weight: コスト関数の重み. error * weight^2 * error. 0の成分はjacobianやerrorに含まれない. eval座標系
    const cnoid::BodyPtr& robot() const { return robot_;}
    cnoid::BodyPtr& robot() { return robot_;}
    const double& weight() const { return weight_;}
    double& weight() { return weight_;}
    const double& g() const { return g_;}
    double& g() { return g_;}

    // 内部状態更新
    void update (const std::vector<cnoid::LinkPtr>& joints, const std::vector<std::shared_ptr<Force> >& forces) override;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    cnoid::BodyPtr robot_;
    double weight_ = 1.0;
    double g_ = 9.80665;

    cnoid::BodyPtr jacobian_robot_ = nullptr;// 前回のjacobian計算時のrobot
    std::vector<cnoid::LinkPtr> jacobian_joints_; // 前回のjacobian計算時のjoints
    std::vector<std::shared_ptr<Force> > jacobian_forces_; // 前回のjacobian計算時のforces

    bool hasJoints_ = true; // jointsにrobotの関節が含まれているか
    Eigen::SparseMatrix<double,Eigen::RowMajor> CMJacobian_;
    std::unordered_map<cnoid::LinkPtr,int> CMJacobian_ColMap_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> AMJacobian_;
    std::unordered_map<cnoid::LinkPtr,int> AMJacobian_ColMap_;
    std::unordered_map<std::shared_ptr<Force>,int> jacobian_forces_ColMap_;

    Eigen::SparseMatrix<double,Eigen::ColMajor> forcejacobian_;
  };
}

#endif
