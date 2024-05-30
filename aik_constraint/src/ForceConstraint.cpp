#include <aik_constraint/ForceConstraint.h>

namespace aik_constraint {
  void ForceConstraint::update (const std::vector<cnoid::LinkPtr>& joints, const std::vector<std::shared_ptr<Force> >& forces) {
    if(!this->force_
       || this->force_->DOF() != this->C_.cols()
       || this->C_.rows() != this->dl_.rows()
       || this->C_.rows() != this->du_.rows() ) {
      std::cerr << __FUNCTION__ << "dimension mismatch" << std::endl;
      this->eq_.resize(0);
      this->jacobian_.resize(0,0);
      this->minIneq_.resize(0);
      this->maxIneq_.resize(0);
      this->jacobianIneq_.resize(0,0);
      this->jacobian_joints_.resize(0);
      this->jacobian_forces_.resize(0);
      this->jacobian_force_ = nullptr;
      this->jacobianIneq_ColMajor_.resize(0,0);
      return;
    }

    cnoid::VectorX current_value = this->C_ * this->force_->F();
    cnoid::VectorX current_lower = this->dl_ - current_value;
    cnoid::VectorX current_upper = this->du_ - current_value;

    this->minIneq_ = current_lower * this->weight_;
    this->maxIneq_ = current_upper * this->weight_;

    // calc jacobian
    if(!this->isJointsSame(joints,this->jacobian_joints_) ||
       !this->isForcesSame(forces,this->jacobian_forces_) ||
       this->force_ != this->jacobian_force_ ||
       this->jacobianIneq_ColMajor_.rows() != this->C_.rows()){
      this->jacobian_joints_ = joints;
      this->jacobian_forces_ = forces;
      this->jacobian_force_ = this->force_;
      this->jacobianColMap_.clear();
      int cols = 0;
      for(size_t i=0; i < this->jacobian_joints_.size(); i++){
        cols += this->getJointDOF(this->jacobian_joints_[i]);
      }
      for(size_t i=0; i < this->jacobian_forces_.size(); i++){
        this->jacobianColMap_[this->jacobian_forces_[i]] = cols;
        cols += this->jacobian_forces_[i]->DOF();
      }

      this->jacobianIneq_ColMajor_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(this->C_.rows(),cols);
    }

    if(this->jacobianColMap_.find(this->jacobian_force_) != this->jacobianColMap_.end()){
      this->jacobianIneq_ColMajor_.middleCols(this->jacobianColMap_[this->jacobian_force_], this->jacobian_force_->DOF()) = this->C_ * this->weight_;
    }
    this->jacobianIneq_ = this->jacobianIneq_ColMajor_;

    if(this->debugLevel_>=2){
      std::cerr << "ForceConstraint" << std::endl;
      std::cerr << "F" << std::endl;
      std::cerr << this->jacobian_force_->F().transpose() << std::endl;
      std::cerr << "minineq" << std::endl;
      std::cerr << this->minIneq_ << std::endl;
      std::cerr << "maxineq" << std::endl;
      std::cerr << this->maxIneq_ << std::endl;
      std::cerr << "jacobianIneq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;
  }
};
