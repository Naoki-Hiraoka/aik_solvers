#ifndef AIK_CONSTRAINT_ForceCONSTRAINT_H
#define AIK_CONSTRAINT_ForceCONSTRAINT_H

#include <aik_constraint/IKConstraint.h>
#include <cnoid/EigenUtil>
#include <iostream>

namespace aik_constraint{
  class ForceConstraint : public IKConstraint
  {
  public:
    // forceの各自由度の値に対する制約
    //  weight: コスト関数の重み. error * weight^2 * error. 0の成分はjacobianやerrorに含まれない. eval座標系

    const std::shared_ptr<Force>& force() const { return force_;}
    std::shared_ptr<Force>& force() { return force_;}
    const double& weight() const { return weight_;}
    double& weight() { return weight_;}
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& C() const { return C_;}
    Eigen::SparseMatrix<double,Eigen::RowMajor>& C() { return C_;}
    const cnoid::VectorX& dl() const { return dl_;}
    cnoid::VectorX& dl() { return dl_;}
    const cnoid::VectorX& du() const { return du_;}
    cnoid::VectorX& du() { return du_;}

    // 内部状態更新
    void update (const std::vector<cnoid::LinkPtr>& joints, const std::vector<std::shared_ptr<Force> >& forces) override;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    std::shared_ptr<Force> force_ = nullptr;
    double weight_ = 1.0;

    Eigen::SparseMatrix<double,Eigen::RowMajor> C_;
    Eigen::VectorXd dl_;
    Eigen::VectorXd du_;

    std::shared_ptr<Force> jacobian_force_ = nullptr;// 前回のjacobian計算時のforce
    std::vector<cnoid::LinkPtr> jacobian_joints_; // 前回のjacobian計算時のjoints
    std::vector<std::shared_ptr<Force> > jacobian_forces_; // 前回のjacobian計算時のforces
    std::unordered_map<std::shared_ptr<Force>,int> jacobianColMap_;
    Eigen::SparseMatrix<double,Eigen::ColMajor> jacobianIneq_ColMajor_;
  };
}

#endif
