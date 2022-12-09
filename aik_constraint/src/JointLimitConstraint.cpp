#include <aik_constraint/JointLimitConstraint.h>
#include <iostream>

namespace aik_constraint{
  void JointLimitConstraint::update (const std::vector<cnoid::LinkPtr>& joints) {
    if(!this->joint_ || !(this->joint_->isRotationalJoint() || this->joint_->isPrismaticJoint())) {
      this->eq_.resize(0);
      this->jacobian_.resize(0,0);
      this->minIneq_.resize(0);
      this->maxIneq_.resize(0);
      this->jacobianIneq_.resize(0,0);
      return;
    }

    double target_acc_lower = 0.0;
    target_acc_lower += std::min(this->pgain_ * (this->joint_->q_lower() - this->joint_->q()), this->maxAccByPosError_);
    target_acc_lower += std::min(this->dgain_ * ( - this->joint_->dq()), this->maxAccByVelError_);
    target_acc_lower = std::min(target_acc_lower, this->maxAcc_);

    double target_acc_upper = 0.0;
    target_acc_upper += std::max(this->pgain_ * (this->joint_->q_upper() - this->joint_->q()), -this->maxAccByPosError_);
    target_acc_upper += std::max(this->dgain_ * ( - this->joint_->dq()), -this->maxAccByVelError_);
    target_acc_upper = std::max(target_acc_upper, -this->maxAcc_);

    if(this->minIneq_.rows() != 1) this->minIneq_ = Eigen::VectorXd(1);
    this->minIneq_[0] = target_acc_lower * this->weight_;
    if(this->maxIneq_.rows() != 1) this->maxIneq_ = Eigen::VectorXd(1);
    this->maxIneq_[0] = target_acc_upper * this->weight_;


    // calc jacobian
    if(!this->isJointsSame(joints,this->jacobianIneq_joints_) ||
       this->joint_ != this->jacobianIneq_joint_){
      this->jacobianIneq_joints_ = joints;
      this->jacobianIneq_joint_ = this->joint_;
      this->jacobianIneqColMap_.clear();
      int cols = 0;
      for(size_t i=0; i < this->jacobianIneq_joints_.size(); i++){
        this->jacobianIneqColMap_[this->jacobianIneq_joints_[i]] = cols;
        cols += this->getJointDOF(this->jacobianIneq_joints_[i]);
      }

      this->jacobianIneq_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(1,cols);

      if(this->jacobianIneqColMap_.find(this->jacobianIneq_joint_) != this->jacobianIneqColMap_.end()){
        this->jacobianIneq_.insert(0,this->jacobianIneqColMap_[this->jacobianIneq_joint_]) = 1;
      }

    }

    if(this->jacobianIneqColMap_.find(this->jacobianIneq_joint_) != this->jacobianIneqColMap_.end()){
      this->jacobianIneq_.coeffRef(0,this->jacobianIneqColMap_[this->jacobianIneq_joint_]) = this->weight_;
    }


    if(this->debugLevel_>=1){
      std::cerr << "JointLimitConstraint" << std::endl;
      std::cerr << "q_lower q q_upper dq" << std::endl;
      std::cerr << this->joint_->q_lower() << " " << this->joint_->q() << " " << this->joint_->q_upper() << " " << this->joint_->dq() << std::endl;
      std::cerr << "minineq maxineq" << std::endl;
      std::cerr << this->minIneq_ << " " << this->maxIneq_ << std::endl;
      std::cerr << "jacobianIneq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;
  }

}
