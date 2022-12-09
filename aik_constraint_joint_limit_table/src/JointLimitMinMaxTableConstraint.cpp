#include <aik_constraint_joint_limit_table/JointLimitMinMaxTableConstraint.h>
#include <iostream>

namespace aik_constraint_joint_limit_table{
  double JointLimitMinMaxTableConstraint::get_q_lower() { // this->joint_はnullptrではない前提
    double lower = this->joint_->q_lower();
    for(size_t i=0;i<this->jointLimitTables_.size();i++){
      if(this->jointLimitTables_[i]->getSelfJoint() == this->joint_){
        lower = std::max(lower, this->jointLimitTables_[i]->getLlimit());
      }
    }

    return lower;
  }
  double JointLimitMinMaxTableConstraint::get_q_upper() { // this->joint_はnullptrではない前提
    double upper = this->joint_->q_upper();
    for(size_t i=0;i<this->jointLimitTables_.size();i++){
      if(this->jointLimitTables_[i]->getSelfJoint() == this->joint_){
        upper = std::min(upper, this->jointLimitTables_[i]->getUlimit());
      }
    }

    return upper;
  }
}
