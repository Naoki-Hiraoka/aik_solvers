#include <aik_constraint/EOMConstraint.h>
#include <aik_constraint/Jacobian.h>
#include <cnoid/Jacobian>

namespace aik_constraint {
  void EOMConstraint::update (const std::vector<cnoid::LinkPtr>& joints, const std::vector<std::shared_ptr<Force> >& forces) {
    if(!this->robot_) {
      this->eq_.resize(0);
      this->jacobian_.resize(0,0);
      this->minIneq_.resize(0);
      this->maxIneq_.resize(0);
      this->jacobianIneq_.resize(0,0);
      this->jacobian_joints_.resize(0);
      this->jacobian_forces_.resize(0);
      this->jacobian_robot_ = nullptr;
      return;
    }

    /*
      M ddq + C * dq + mg = \sigma G * F
      - M ddq + \sigma G * F = C * dq + mg

      F_extには、現在の M ddq + C * dqが入っている. (ただしroot周り)
     */
    this->eq_ = cnoid::Vector6::Zero(); // 並進[N]、回転[Nm]. world系. 重心周り.
    this->eq_[2] += this->robot_->mass() * this->g_ * this->weight_; // mg

    // calc jacobian
    if(!this->isJointsSame(joints,this->jacobian_joints_) ||
       !this->isForcesSame(forces,this->jacobian_forces_) ||
       this->robot_ != this->jacobian_robot_ ){
      this->jacobian_joints_ = joints;
      this->jacobian_forces_ = forces;
      this->jacobian_robot_ = this->robot_;
      this->CMJacobian_ColMap_.clear();
      this->AMJacobian_ColMap_.clear();
      this->jacobian_forces_ColMap_.clear();

      aik_constraint::calcCMJacobianShape(this->jacobian_joints_,
                                          this->jacobian_forces_,
                                          this->jacobian_robot_,
                                          nullptr,
                                          this->CMJacobian_,
                                          this->CMJacobian_ColMap_);
      aik_constraint::calcAngularMomentumJacobianShape(this->jacobian_joints_,
                                                       this->jacobian_forces_,
                                                       this->jacobian_robot_,
                                                       nullptr,
                                                       this->AMJacobian_,
                                                       this->AMJacobian_ColMap_);
      int cols = 0;
      for(size_t i=0; i < this->jacobian_joints_.size(); i++){
        cols += this->getJointDOF(this->jacobian_joints_[i]);
      }
      for(size_t i=0; i < this->jacobian_forces_.size(); i++){
        this->jacobian_forces_ColMap_[this->jacobian_forces_[i]] = cols;
        cols += this->jacobian_forces_[i]->DOF();
      }

      this->jacobian_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(6,cols);
      this->forcejacobian_ = Eigen::SparseMatrix<double,Eigen::ColMajor>(6,cols);
    }

    Eigen::MatrixXd CMJ;
    if(this->robot_) cnoid::calcCMJacobian(this->robot_,nullptr,CMJ); // [joint root]の順
    aik_constraint::calcCMJacobianCoef(this->jacobian_joints_,
                                       this->jacobian_forces_,
                                       this->jacobian_robot_,
                                       nullptr,
                                       CMJ,// not used
                                       CMJ,//tmp
                                       this->CMJacobian_ColMap_,
                                       this->CMJacobian_);
    this->jacobian_.topRows<3>() = - this->CMJacobian_ * this->robot_->mass() * this->weight_;
    this->eq_.head<3>() += this->robot_->rootLink()->F_ext().head<3>() * this->weight_;

    Eigen::MatrixXd AMJ;
    cnoid::calcAngularMomentumJacobian(this->robot_,nullptr,AMJ); // [joint root]の順. comまわり
    aik_constraint::calcAngularMomentumJacobianCoef(this->jacobian_joints_,
                                                    this->jacobian_forces_,
                                                    this->jacobian_robot_,
                                                    nullptr,
                                                    AMJ,
                                                    AMJ,// not used
                                                    this->AMJacobian_ColMap_,
                                                    this->AMJacobian_);
    this->jacobian_.bottomRows<3>() = - this->AMJacobian_ * this->weight_;
    this->eq_.tail<3>() += (this->robot_->rootLink()->F_ext().tail<3>()/*root周り*/
                            + (this->robot_->rootLink()->p() - this->robot_->centerOfMass()).cross(this->robot_->rootLink()->F_ext().head<3>())) * this->weight_;

    for(int i=0;i<this->jacobian_forces_.size();i++){
      double sign = 0.0;
      if(this->jacobian_forces_[i]->A_link() && this->jacobian_forces_[i]->A_link()->body() == this->robot_) sign += 1.0;
      if(this->jacobian_forces_[i]->B_link() && this->jacobian_forces_[i]->B_link()->body() == this->robot_) sign += -1.0;
      if(sign==0.0) continue;
      Eigen::SparseMatrix<double,Eigen::ColMajor> GraspMatrix(6,6);
      {
        const cnoid::Isometry3d pose = this->jacobian_forces_[i]->A_link() ? this->jacobian_forces_[i]->A_link()->R() * this->jacobian_forces_[i]->A_localpos() : this->jacobian_forces_[i]->A_localpos();
        const Eigen::Matrix3d& R = pose.linear();
        const Eigen::Matrix3d& p_x_R = aik_constraint::hat(pose.translation() - this->robot_->centerOfMass()) * R;
        /*
          |R   0|
            |pxR R|
        */
        for(int k=0;k<3;k++){
          for(int j=0;j<3;j++) GraspMatrix.insert(j,k) = R(j,k);
          for(int j=0;j<3;j++) GraspMatrix.insert(3+j,k) = p_x_R(j,k);
        }
        for(int k=0;k<3;k++){
          for(int j=0;j<3;j++) GraspMatrix.insert(3+j,3+k) = R(j,k);
        }
      }
      Eigen::SparseMatrix<double,Eigen::ColMajor> J = sign * GraspMatrix * this->jacobian_forces_[i]->S();
      this->forcejacobian_.middleCols(this->jacobian_forces_ColMap_[this->jacobian_forces_[i]],this->jacobian_forces_[i]->DOF()) = J;
      this->eq_ -= J * this->jacobian_forces_[i]->F() * this->weight_;
    }
    this->jacobian_ += Eigen::SparseMatrix<double,Eigen::RowMajor>(this->forcejacobian_) * this->weight_;

    if(this->debugLevel_>=2){
      std::cerr << "EOMConstraint" << std::endl;
      std::cerr << "eq" << std::endl;
      std::cerr << this->eq_ << std::endl;
      std::cerr << "jacobian" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
    }

    return;
  }
};
