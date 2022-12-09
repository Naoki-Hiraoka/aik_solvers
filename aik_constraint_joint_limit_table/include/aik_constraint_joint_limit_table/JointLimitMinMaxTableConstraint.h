#ifndef AIK_CONSTRAINT_JOINTLIMITMINMAXTABLECONSTRAINT_H
#define AIK_CONSTRAINT_JOINTLIMITMINMAXTABLECONSTRAINT_H

#include <aik_constraint/JointLimitConstraint.h>
#include <joint_limit_table/JointLimitTable.h>

namespace aik_constraint_joint_limit_table{
  class JointLimitMinMaxTableConstraint : public aik_constraint::JointLimitConstraint {
  public:
    //jointのqをq_upperとq_lowerの間かつmin-max-tableの間にさせる.

    const std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> >& jointLimitTables() const { return jointLimitTables_;}
    std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> >& jointLimitTables() { return jointLimitTables_;}

  protected:
    double get_q_lower() override; // this->joint_はnullptrではない前提
    double get_q_upper() override; // this->joint_はnullptrではない前提

    std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > jointLimitTables_;
  };
}

#endif
