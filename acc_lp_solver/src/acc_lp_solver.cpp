#include <acc_lp_solver/acc_lp_solver.h>
#include <clpeigen/clpeigen.h>
#include <Eigen/Sparse>
#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <cnoid/TimeMeasure>

namespace acc_lp_solver {
  bool solveLP (const std::shared_ptr<aik_constraint::Force>& force,
                const std::vector<cnoid::LinkPtr>& variables,
                const std::vector<std::shared_ptr<aik_constraint::Force> >& forces,
                const std::vector<std::shared_ptr<aik_constraint::IKConstraint> >& ikc_list,
                const IKParam& param
                ) {

    // for debug
    cnoid::TimeMeasure timer;
    if(param.debugLevel>=1) timer.begin();

    for ( int i=0; i<ikc_list.size(); i++ ) {
      ikc_list[i]->update(variables,forces);
    }

    int dim = 0;
    int ddqdim = 0;
    int forcedim = 0;
    int target = -1;
    for(size_t i=0;i<variables.size();i++) {
      dim+=aik_constraint::IKConstraint::getJointDOF(variables[i]);
      ddqdim+=aik_constraint::IKConstraint::getJointDOF(variables[i]);
    }
    for(size_t i=0;i<forces.size();i++) {
      if(forces[i] == force) target = dim;
      dim+=forces[i]->DOF();
      forcedim+=forces[i]->DOF();
    }


    int num_eqs = 0;
    int num_ineqs = 0;
    for(size_t i=0; i<ikc_list.size(); i++){
      num_eqs += ikc_list[i]->getJacobian().rows();
      num_ineqs += ikc_list[i]->getJacobianIneq().rows();

      if((ikc_list[i]->getEq().rows() != ikc_list[i]->getJacobian().rows()) ||
         (ikc_list[i]->getJacobian().rows() > 0 && dim != ikc_list[i]->getJacobian().cols()) ||
         (ikc_list[i]->getMinIneq().rows() != ikc_list[i]->getJacobianIneq().rows()) ||
         (ikc_list[i]->getMaxIneq().rows() != ikc_list[i]->getJacobianIneq().rows()) ||
         (ikc_list[i]->getJacobianIneq().rows() > 0 && dim != ikc_list[i]->getJacobianIneq().cols())){
        std::cerr << __FUNCTION__ << " dimension mismatch" << std::endl;
        return false;
      }
    }

    Eigen::VectorXd o = - Eigen::VectorXd::Zero(dim);
    Eigen::SparseMatrix<double,Eigen::RowMajor> A(num_eqs + num_ineqs, dim);
    Eigen::VectorXd lbA = Eigen::VectorXd::Zero(A.rows());
    Eigen::VectorXd ubA = Eigen::VectorXd::Zero(A.rows());
    Eigen::VectorXd lb(A.cols()); for(size_t i=0;i<lb.rows();i++) lb[i] = -param.maxValue;
    Eigen::VectorXd ub(A.cols()); for(size_t i=0;i<ub.rows();i++) ub[i] = param.maxValue;

    {
      int idx = 0;
      for(size_t i=0;i<ikc_list.size(); i++){
        A.middleRows(idx,ikc_list[i]->getJacobian().rows()) = ikc_list[i]->getJacobian();
        lbA.segment(idx,ikc_list[i]->getEq().rows()) = ikc_list[i]->getEq();
        ubA.segment(idx,ikc_list[i]->getEq().rows()) = ikc_list[i]->getEq();
        idx += ikc_list[i]->getJacobian().rows();
        A.middleRows(idx,ikc_list[i]->getJacobianIneq().rows()) = ikc_list[i]->getJacobianIneq();
        lbA.segment(idx,ikc_list[i]->getMinIneq().rows()) = ikc_list[i]->getMinIneq();
        ubA.segment(idx,ikc_list[i]->getMaxIneq().rows()) = ikc_list[i]->getMaxIneq();
        idx += ikc_list[i]->getJacobianIneq().rows();
      }
    }

    if(target != -1) {
      for(int i=0;i<force->DOF();i++) o[target+i] = 1.0;
    }

    if(param.debugLevel >= 2){
      std::cerr << "[" <<__FUNCTION__ << "]" << std::endl;
      std::cerr << "o" << std::endl;
      std::cerr << o << std::endl;
      std::cerr << "A" << std::endl;
      std::cerr << A << std::endl;
      std::cerr << "ubA" << std::endl;
      std::cerr << ubA << std::endl;
      std::cerr << "lbA" << std::endl;
      std::cerr << lbA << std::endl;
    }


    clpeigen::solver solver;
    solver.model().setPrimalTolerance(param.lpTolerance);//default 1e-7. 1e-12などにするとvertexを見逃さないようにする
    solver.model().setDualTolerance(param.lpTolerance);
    solver.initialize(o,A,lbA,ubA,lb,ub,(param.debugLevel>=2) ? 1 : 0);
    bool solved =  solver.solve();

    if(!solved){
      std::cerr << __FUNCTION__ << " failed" << std::endl;
      return false;
    }

    Eigen::VectorXd result;
    solver.getSolution(result);
    if(!result.allFinite()){
      std::cerr << __FUNCTION__ << " nax/inf is found" << std::endl;
      return false;
    }


    if(param.debugLevel >= 2){
      std::cerr << "result" << std::endl;
      std::cerr << result.transpose() << std::endl;
    }

    {
      size_t idx = 0;
      for(size_t i=0;i<variables.size();i++){
        if(variables[i]->isRevoluteJoint() || variables[i]->isPrismaticJoint()){
          // update joint angles
          variables[i]->ddq() += result[idx];
        }else if(variables[i]->isFreeJoint()) {
          // update rootlink pos rot
          variables[i]->dv() += result.segment<3>(idx);
          variables[i]->dw() += result.segment<3>(idx+3);
        }

        idx += aik_constraint::IKConstraint::getJointDOF(variables[i]);
      }
      for(size_t i=0;i<forces.size();i++){
        forces[i]->F() += result.segment(idx,forces[i]->DOF());
        idx += forces[i]->DOF();
      }
    }

    if(param.debugLevel>=1) {
      double time = timer.measure();
      std::cerr << __FUNCTION__ << " time: " << time << "[s]" << std::endl;
    }

    return true;
  }

}
