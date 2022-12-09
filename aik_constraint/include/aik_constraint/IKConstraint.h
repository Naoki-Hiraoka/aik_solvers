#ifndef AIK_CONSTRAINT_IKCONSTRAINT_H
#define AIK_CONSTRAINT_IKCONSTRAINT_H

#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include <Eigen/Sparse>
#include <unordered_map>

namespace aik_constraint{
  class IKConstraint
  {
  public:

    // 必ず,状態更新(全リンクのF_extは0. 探索変数のリンクのddqは0) -> ForwardKinematics(true,true) -> calcCenterOfMass() -> calcInverseDynamics()してrootLinkが受ける力をrootLink->F_ext()に入れる -> update() -> getError() / getJacobian / getMin/MaxIneq / getJacobianIneq / getDrawOnObjects の順で呼ぶので、同じ処理を何度も行うのではなく最初に呼ばれる関数で1回だけ行って以降はキャッシュを使ってよい

    // 内部状態更新
    virtual void update (const std::vector<cnoid::LinkPtr>& joints) { return; }

    // for debug view
    virtual const std::vector<cnoid::SgNodePtr>& getDrawOnObjects() { return this->drawOnObjects_; }

    // getEq = getJacobian * ddq
    // 等式制約のエラーを返す.
    const Eigen::VectorXd& getEq () const { return this->eq_; }
    // 等式制約のヤコビアンを返す. 各jointsの加速度が変数
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& getJacobian () const { return this->jacobian_; }

    // getMinIneq <= getJacobianIneq * ddq <= getMaxIneq()
    // 不等式制約のmin値を返す
    const Eigen::VectorXd& getMinIneq () const { return this->minIneq_; }
    // 不等式制約のmax値を返す
    const Eigen::VectorXd& getMaxIneq () const { return this->maxIneq_; }
    // 不等式制約のヤコビアンを返す. 各jointsの加速度が変数
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& getJacobianIneq () const { return this->jacobianIneq_; }

    const int& debugLevel() const { return debugLevel_;}
    int& debugLevel() { return debugLevel_;}

    static size_t getJointDOF(const cnoid::LinkPtr& joint);
    static bool isJointsSame(const std::vector<cnoid::LinkPtr>& joints1,const std::vector<cnoid::LinkPtr>& joints2);
    static double clamp(const double& value, const double& limit_value) {
      return std::min(std::max(value, -limit_value), limit_value);
    }
    template<typename Derived>
    static typename Derived::PlainObject clamp(const Eigen::MatrixBase<Derived>& value, const Eigen::MatrixBase<Derived>& limit_value) {
      return value.array().max(-limit_value.array()).min(limit_value.array());
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  protected:

    int debugLevel_ = 0;
    std::vector<cnoid::SgNodePtr> drawOnObjects_;

    Eigen::VectorXd eq_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_;
    Eigen::VectorXd minIneq_;
    Eigen::VectorXd maxIneq_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobianIneq_;
  };
}

#endif
