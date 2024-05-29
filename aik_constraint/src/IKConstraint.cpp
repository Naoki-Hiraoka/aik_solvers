#include <aik_constraint/IKConstraint.h>

namespace aik_constraint{
  bool IKConstraint::isJointsSame(const std::vector<cnoid::LinkPtr>& joints1,const std::vector<cnoid::LinkPtr>& joints2){
    if (joints1.size() != joints2.size() ) return false;
    for(size_t i=0;i<joints1.size();i++){
      if (joints1[i] != joints2[i] ) return false;
    }
    return true;
  }

  bool IKConstraint::isForcesSame(const std::vector<std::shared_ptr<Force> >& forces1,const std::vector<std::shared_ptr<Force> >& forces2) {
    if (forces1.size() != forces2.size() ) return false;
    for(size_t i=0;i<forces1.size();i++){
      if (forces1[i] != forces2[i] ) return false;
    }
    return true;
  }

  size_t IKConstraint::getJointDOF(const cnoid::LinkPtr& joint) {
    if(joint->isRevoluteJoint() || joint->isPrismaticJoint()) return 1;
    else if(joint->isFreeJoint()) return 6;
    else return 0;
  }

}
