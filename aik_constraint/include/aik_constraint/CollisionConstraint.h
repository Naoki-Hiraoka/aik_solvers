#ifndef AIK_CONSTRAINT_COLLISIONCONSTRAINT_H
#define AIK_CONSTRAINT_COLLISIONCONSTRAINT_H

#include <aik_constraint/IKConstraint.h>

namespace aik_constraint{
  class CollisionConstraint : public IKConstraint
  {
  public:

    // A_linkとB_linkの干渉を回避する
    //  tolerance: この値以上離す[m]
    //  maxError: エラーの頭打ち
    //  precision: 収束判定の閾値
    //  weight: コスト関数の重み. error * weight^2 * error. maxErrorの適用後に適用する
    //  velocityDamper: 不等式制約の差分をこの値分の1にする. maxErrorの適用前に適用する.
    //状態が更新される度に, 手動でcalcForwardKinematics()を呼ぶ必要が有る.

    const cnoid::LinkPtr& A_link() const { return A_link_;}
    cnoid::LinkPtr& A_link() { return A_link_;}
    const cnoid::LinkPtr& B_link() const { return B_link_;}
    cnoid::LinkPtr& B_link() { return B_link_;}
    const double& tolerance() const { return tolerance_;}
    double& tolerance() { return tolerance_;}
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

    // for debug view
    const std::vector<cnoid::SgNodePtr>& getDrawOnObjects() override;

  protected:
    //A_v, B_vはlocal系
    virtual bool computeDistance(const cnoid::LinkPtr A_link, const cnoid::LinkPtr B_link, double& distance, cnoid::Vector3& direction/*B->A*/, cnoid::Vector3& A_v, cnoid::Vector3& B_v)=0;

  private:
    cnoid::LinkPtr A_link_ = nullptr;
    cnoid::LinkPtr B_link_ = nullptr;
    double tolerance_ = 0.01;
    double pgain_ = 400;
    double dgain_ = 50;
    double maxAcc_ = 15;
    double maxAccByPosError_ = 5;
    double maxAccByVelError_ = 10;
    double weight_ = 1.0;

    cnoid::SgLineSetPtr lines_;

    cnoid::Vector3 A_currentLocalp_ = cnoid::Vector3::Zero();
    cnoid::Vector3 B_currentLocalp_ = cnoid::Vector3::Zero();
    cnoid::Vector3 currentDirection_ = cnoid::Vector3::UnitX(); // B->A

    std::vector<cnoid::LinkPtr> path_A_joints_;
    std::vector<cnoid::LinkPtr> path_B_joints_;
    std::vector<cnoid::LinkPtr> path_BA_joints_;
    int path_BA_joints_numUpwardConnections_ = 0;
    cnoid::LinkPtr jacobianIneq_A_link_ = nullptr;// 前回のjacobian計算時のA_link
    cnoid::LinkPtr jacobianIneq_B_link_ = nullptr;// 前回のjacobian計算時のB_link
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobianIneq_full_;
    std::vector<cnoid::LinkPtr> jacobianIneq_joints_; // 前回のjacobian計算時のjoints
    std::unordered_map<cnoid::LinkPtr,int> jacobianIneqColMap_;

  };


}

#endif
