#include <aik_constraint/JointAngleConstraint.h>
#include <iostream>

namespace aik_constraint{
  void JointAngleConstraint::update (const std::vector<cnoid::LinkPtr>& joints, const std::vector<std::shared_ptr<Force> >& forces) {
    if(!this->joint_ || !(this->joint_->isRevoluteJoint() || this->joint_->isPrismaticJoint())) {
      this->eq_.resize(0);
      this->jacobian_.resize(0,0);
      this->jacobian_joint_ = nullptr;
      this->jacobian_joints_.resize(0);
      this->jacobian_forces_.resize(0);
      this->minIneq_.resize(0);
      this->maxIneq_.resize(0);
      this->jacobianIneq_.resize(0,0);
      return;
    }

    double target_acc = 0.0;
    target_acc += this->ref_acc_;
    target_acc += this->clamp(this->pgain_ * (targetq_ - this->joint_->q()), this->maxAccByPosError_);
    target_acc += this->clamp(this->dgain_ * (targetdq_ - this->joint_->dq()), this->maxAccByVelError_);
    target_acc -= this->joint_->ddq();
    target_acc = this->clamp(target_acc, this->maxAcc_);

    if(this->eq_.rows() != 1) this->eq_ = Eigen::VectorXd(1);
    this->eq_[0] = this->weight_ * target_acc;

    if(!this->isJointsSame(joints,this->jacobian_joints_) ||
       !this->isForcesSame(forces,this->jacobian_forces_) ||
       this->joint_ != this->jacobian_joint_){
      this->jacobian_joints_ = joints;
      this->jacobian_forces_ = forces;
      this->jacobian_joint_ = this->joint_;
      this->jacobianColMap_.clear();
      int cols = 0;
      for(size_t i=0; i < this->jacobian_joints_.size(); i++){
        this->jacobianColMap_[this->jacobian_joints_[i]] = cols;
        cols += this->getJointDOF(this->jacobian_joints_[i]);
      }
      for(size_t i=0; i < this->jacobian_forces_.size(); i++){
        cols += this->jacobian_forces_[i]->DOF();
      }

      this->jacobian_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(1,cols);

      if(this->jacobianColMap_.find(this->jacobian_joint_) != this->jacobianColMap_.end()){
        this->jacobian_.insert(0,this->jacobianColMap_[this->jacobian_joint_]) = 1;
      }

    }

    if(this->jacobianColMap_.find(this->jacobian_joint_) != this->jacobianColMap_.end()){
      this->jacobian_.coeffRef(0,this->jacobianColMap_[this->jacobian_joint_]) = this->weight_;
    }


    if(this->debugLevel_>=2){
      std::cerr << "JointAngleConstraint " << ((this->joint_)?this->joint_->name():"") <<  std::endl;
      std::cerr << "q dq targetq targetdq ref_acc" << std::endl;
      std::cerr << this->joint_->q() << " " << this->joint_->dq() << " " << this->targetq_ << " " << this->targetdq_ << " " << this->ref_acc_ << std::endl;
      std::cerr << "eq" << std::endl;
      std::cerr << this->eq_ << std::endl;
      std::cerr << "jacobian" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
    }

    return;
  }
}
