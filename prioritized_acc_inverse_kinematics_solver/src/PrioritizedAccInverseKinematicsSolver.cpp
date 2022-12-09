#include <prioritized_acc_inverse_kinematics_solver/PrioritizedAccInverseKinematicsSolver.h>
#include <Eigen/Sparse>
#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <cnoid/TimeMeasure>

namespace prioritized_acc_inverse_kinematics_solver {
  bool solveAIK (const std::vector<cnoid::LinkPtr>& variables,
                 const std::vector<std::vector<std::shared_ptr<aik_constraint::IKConstraint> > >& ikc_list,
                 std::vector<std::shared_ptr<prioritized_qp_base::Task> >& prevTasks,
                 const IKParam& param,
                 std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)> taskGeneratorFunc) {

    // for debug
    cnoid::TimeMeasure timer;
    if(param.debugLevel>0) timer.begin();

    for ( int i=0; i<ikc_list.size(); i++ ) {
      for(size_t j=0;j<ikc_list[i].size(); j++){
        ikc_list[i][j]->update(variables);
      }
    }

    // Solvability-unconcerned Inverse Kinematics by Levenberg-Marquardt Method [sugihara:RSJ2009]
    // H = J^T * We * J + Wn
    // Wn = (e^T * We * e + \bar{wn}) * Wq // Wq: modify to insert dq weight

    double dim = 0;
    for(size_t i=0;i<variables.size();i++) dim+=aik_constraint::IKConstraint::getJointDOF(variables[i]);

    if(prevTasks.size() != ikc_list.size()) {
      prevTasks.clear();
      prevTasks.resize(ikc_list.size(),nullptr);
    }
    for(size_t i=0;i<ikc_list.size();i++){
      taskGeneratorFunc(prevTasks[i],param.debugLevel);

      if(i!=0) prevTasks[i]->toSolve() = true;
      else prevTasks[i]->toSolve() = false;

      int num_eqs = 0;
      int num_ineqs = 0;
      std::vector<std::reference_wrapper<const Eigen::VectorXd> > eqs;eqs.reserve(ikc_list[i].size());
      std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > > jacobians;jacobians.reserve(ikc_list[i].size());
      std::vector<std::reference_wrapper <const Eigen::VectorXd> > minineqs;minineqs.reserve(ikc_list[i].size());
      std::vector<std::reference_wrapper<const Eigen::VectorXd> > maxineqs;maxineqs.reserve(ikc_list[i].size());
      std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > > jacobianineqs;jacobianineqs.reserve(ikc_list[i].size());

      for(size_t j=0; j<ikc_list[i].size(); j++){
        eqs.emplace_back(ikc_list[i][j]->getEq());
        jacobians.emplace_back(ikc_list[i][j]->getJacobian());
        jacobianineqs.emplace_back(ikc_list[i][j]->getJacobianIneq());
        minineqs.emplace_back(ikc_list[i][j]->getMinIneq());
        maxineqs.emplace_back(ikc_list[i][j]->getMaxIneq());

        num_eqs += eqs[j].get().rows();
        num_ineqs += minineqs[j].get().rows();

        if((eqs[j].get().rows() != jacobians[j].get().rows()) ||
           (jacobians[j].get().rows() > 0 && dim != jacobians[j].get().cols()) ||
           (minineqs[j].get().rows() != jacobianineqs[j].get().rows()) ||
           (maxineqs[j].get().rows() != jacobianineqs[j].get().rows()) ||
           (jacobianineqs[j].get().rows() > 0 && dim != jacobianineqs[j].get().cols())){
          std::cerr << "[prioritized_acc_inverse_kinematics_solver::solveAIK] dimension mismatch" << std::endl;
          return false;
        }
      }

      prevTasks[i]->A().resize(num_eqs, dim);
      prevTasks[i]->b().resize(num_eqs);
      prevTasks[i]->C().resize(num_ineqs, dim);
      prevTasks[i]->dl().resize(num_ineqs);
      prevTasks[i]->du().resize(num_ineqs);
      prevTasks[i]->wa() = cnoid::VectorXd::Ones(num_eqs);
      prevTasks[i]->wc() = cnoid::VectorXd::Ones(num_ineqs);

      int idx_eq = 0;
      int idx_ineq = 0;
      for(size_t j=0;j<ikc_list[i].size(); j++){
        prevTasks[i]->A().middleRows(idx_eq,eqs[j].get().rows()) = jacobians[j].get();
        prevTasks[i]->b().segment(idx_eq,eqs[j].get().rows()) = eqs[j].get();
        idx_eq += eqs[j].get().rows();

        prevTasks[i]->C().middleRows(idx_ineq,minineqs[j].get().rows()) = jacobianineqs[j].get();
        prevTasks[i]->dl().segment(idx_ineq,minineqs[j].get().rows()) = minineqs[j].get();
        prevTasks[i]->du().segment(idx_ineq,minineqs[j].get().rows()) = maxineqs[j].get();
        idx_ineq += minineqs[j].get().rows();
      }

      double sumError = 0;
      sumError += prevTasks[i]->b().squaredNorm();
      for(size_t j=0;j<prevTasks[i]->dl().size(); j++) {
        if(prevTasks[i]->dl()[j]>0) sumError += std::pow(prevTasks[i]->dl()[j],2);
        if(prevTasks[i]->du()[j]<0) sumError += std::pow(prevTasks[i]->du()[j],2);
      }
      prevTasks[i]->w() = cnoid::VectorXd::Ones(dim) * (sumError * ((param.weVec.size()==ikc_list.size())?param.weVec[i]:param.we)+ ((param.wnVec.size()==ikc_list.size())?param.wnVec[i]:param.wn));
      if(param.ddqWeight.size() == dim) {
        for(int j=0;j<dim;j++) prevTasks[i]->w()[j] *= param.ddqWeight[j];
      }

      if(param.debugLevel>0) prevTasks[i]->name() = std::string("Task") + std::to_string(i);
    }

    // solve
    cnoid::VectorX result;
    if(!prioritized_qp_base::solve(prevTasks, result, param.debugLevel)){
      std::cerr <<"[PrioritizedAIK] prioritized_qp_base::solve failed" << std::endl;
      return false;
    }

    if (!result.allFinite()) {
      std::cerr <<"[PrioritizedAIK] ERROR nan/inf is found" << std::endl;
      return false;
    }

    size_t idx = 0;
    for(size_t i=0;i<variables.size();i++){
      if(variables[i]->isRevoluteJoint() || variables[i]->isPrismaticJoint()){
        // update joint angles
        variables[i]->ddq() = result[idx];
      }else if(variables[i]->isFreeJoint()) {
        // update rootlink pos rot
        variables[i]->dv() = result.segment<3>(idx);
        variables[i]->dw() = result.segment<3>(idx+3);
      }

      idx += aik_constraint::IKConstraint::getJointDOF(variables[i]);
    }

    if(param.debugLevel>0) {
      double time = timer.measure();
      std::cerr << "[PrioritizedAIK] solveIKOnce time: " << time << "[s]" << std::endl;
    }

    return true;
  }

}
