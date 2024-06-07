#ifndef AIK_CONSTRAINT_FORCE_H
#define AIK_CONSTRAINT_FORCE_H

#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include <Eigen/Sparse>

namespace aik_constraint {
  class Force {
  public:
    // Jointと同様に、基本的に一度Forceオブジェクトが生成されたら、F以外の値 (A_link, B_link, S, DOF)は変化しないものとする. Forceオブジェクトを指すポインタが前回の周期と同じであれば、これらの値は前回の周期と同じであるとみなしてキャッシュ利用してよい.

    // A_linkとB_linkが, A_linkのA_localposの位置で接触する.
    // nullptrの場合、worldを意味する.
    const cnoid::LinkPtr& A_link() const { return A_link_;}
    cnoid::LinkPtr& A_link() { return A_link_;}
    const cnoid::Isometry3& A_localpos() const { return A_localpos_;}
    cnoid::Isometry3& A_localpos() { return A_localpos_;}
    const cnoid::LinkPtr& B_link() const { return B_link_;}
    cnoid::LinkPtr& B_link() { return B_link_;}

    // S: 6 x N. 接触力を表す変数(N次元)をA_localpos localのA_linkが受ける6次元力(F N)に変換する行列.
    // 例えばFACE表現なら単位行列であり、SPAN表現なら[f1 f2 f3 ...]である
    // 次元を調整したければ、Fの単位を[1000N]or[1000Nm]などとし、Sで適切にスケーリングして[N][Nm]に直す. 同時に、[N][Nm]の次元の各種タスクでweightを1/1000などとする.
    const cnoid::VectorX& F() const { return F_; }
    cnoid::VectorX& F() { return F_; }
    Eigen::SparseMatrix<double,Eigen::RowMajor>& S() { return S_;}
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& S() const { return S_;}

    double DOF() const { return F_.rows(); }
    bool isValid() const { return F_.rows() == S_.cols(); }

    // for debug view
    const std::vector<cnoid::SgNodePtr>& getDrawOnObjects();

    // utility. SとFを初期化する
    static void setFACE(std::shared_ptr<Force>& force);
    static void setSPAN(std::shared_ptr<Force>& force, double mu);
  protected:
    cnoid::LinkPtr A_link_;
    cnoid::Isometry3 A_localpos_ = cnoid::Isometry3::Identity();
    cnoid::LinkPtr B_link_;
    cnoid::VectorX F_ = cnoid::VectorX(0);
    Eigen::SparseMatrix<double,Eigen::RowMajor> S_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(6,0);
    std::vector<cnoid::SgNodePtr> drawOnObjects_;
    cnoid::SgLineSetPtr lines_;
  };
};

#endif
