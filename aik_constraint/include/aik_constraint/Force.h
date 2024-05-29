#ifndef AIK_CONSTRAINT_FORCE_H
#define AIK_CONSTRAINT_FORCE_H

#include <cnoid/Body>

namespace aik_constraint {
  class Force {
  public:
    // A_linkとB_linkが, A_linkのA_localposの位置で接触する.
    // nullptrの場合、worldを意味する.
    const cnoid::LinkPtr& A_link() const { return A_link_;}
    cnoid::LinkPtr& A_link() { return A_link_;}
    const cnoid::Isometry3& A_localpos() const { return A_localpos_;}
    cnoid::Isometry3& A_localpos() { return A_localpos_;}
    const cnoid::LinkPtr& B_link() const { return B_link_;}
    cnoid::LinkPtr& B_link() { return B_link_;}

    // S: 6 x N. 接触力を表す変数(N次元)をA_localpos localのA_linkが受ける6次元力(F N)に変換する行列.
    // 例えばFACE表現なら単位行列であり、SPAN表現なら[n 0]^Tである
    Eigen::SparseMatrix<double,Eigen::RowMajor>& S() { return this->S_;}
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& S() const { return this->S_;}

    double DOF() { return S_.cols(); }
  protected:
    cnoid::LinkPtr A_link_;
    cnoid::Isometry3 A_localpos_;
    cnoid::LinkPtr B_link_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> S_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(6,0);
  };
};

#endif
