#ifndef AIK_CONSTAINT_JACOBIAN_H
#define AIK_CONSTAINT_JACOBIAN_H

#include <vector>
#include <unordered_map>

#include <cnoid/Body>
#include <Eigen/Eigen>
#include <Eigen/Sparse>

namespace aik_constraint {
  // world座標系で見た、A - B のヤコビアン. linkがnullptrの場合、localposはworld座標を意味する.
  //   jacobianを新たにコンストラクトし、非ゼロ要素に1を入れる.
  void calc6DofJacobianShape(const std::vector<cnoid::LinkPtr>& joints, //input
                             cnoid::LinkPtr& A_link, //input
                             cnoid::LinkPtr& B_link, //input
                             const bool& calcRotation, // input. falseならtranslationのみ
                             Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian, //output
                             std::unordered_map<cnoid::LinkPtr,int>& jacobianColMap, //output
                             std::vector<cnoid::LinkPtr>& path_A_joints, //output
                             std::vector<cnoid::LinkPtr>& path_B_joints, //output
                             std::vector<cnoid::LinkPtr>& path_BA_joints, //output
                             int& path_BA_joints_numUpwardConnections //output
                             );
  // world座標系で見た、A - B のヤコビアン. linkがnullptrの場合、localposはworld座標を意味する.
  //   jacobianの形状は上の関数で既に整えられている前提.
  void calc6DofJacobianCoef(const std::vector<cnoid::LinkPtr>& joints, //input
                            const cnoid::LinkPtr& A_link, //input
                            const cnoid::Position& A_localpos, //input
                            const cnoid::LinkPtr& B_link, //input
                            const cnoid::Position& B_localpos, //input
                            std::unordered_map<cnoid::LinkPtr,int>& jacobianColMap, //input
                            const std::vector<cnoid::LinkPtr>& path_A_joints, //input
                            const std::vector<cnoid::LinkPtr>& path_B_joints, //input
                            const std::vector<cnoid::LinkPtr>& path_BA_joints, //input
                            const int& path_BA_joints_numUpwardConnections, //input
                            const bool& calcRotation, // input. falseならtranslationのみ
                            Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian //output
                            );

  // world座標系で見た、A - B のヤコビアン. robotがnullptrの場合、world座標を意味する.
  //   jacobianを新たにコンストラクトし、非ゼロ要素に1を入れる.
  void calcCMJacobianShape(const std::vector<cnoid::LinkPtr>& joints,//input
                                  const cnoid::BodyPtr& A_robot,//input
                                  const cnoid::BodyPtr& B_robot,//input
                                  Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian,//output
                                  std::unordered_map<cnoid::LinkPtr,int>& jacobianColMap //output
                                  );
  // world座標系で見た、A - B のヤコビアン. robotがnullptrの場合、world座標を意味する.
  //   jacobianの形状は上の関数で既に整えられている前提.
  void calcCMJacobianCoef(const std::vector<cnoid::LinkPtr>& joints,//input
                          const cnoid::BodyPtr& A_robot,//input
                          const cnoid::BodyPtr& B_robot,//input
                          const Eigen::MatrixXd& A_CMJ, //[joint root]の順 input
                          const Eigen::MatrixXd& B_CMJ, //[joint root]の順 input
                          std::unordered_map<cnoid::LinkPtr,int>& jacobianColMap, //input
                          Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian//output
                          );

  namespace cnoid18 {
    void calcAngularMomentumJacobian(cnoid::Body* body, cnoid::Link* base, Eigen::MatrixXd& H);
  }

  void calcAngularMomentumJacobianShape(const std::vector<cnoid::LinkPtr>& joints,//input
                                        const cnoid::BodyPtr& A_robot,//input
                                        const cnoid::BodyPtr& B_robot,//input
                                        Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian,//output
                                        std::unordered_map<cnoid::LinkPtr,int>& jacobianColMap //output
                                        );
  void calcAngularMomentumJacobianCoef(const std::vector<cnoid::LinkPtr>& joints,//input
                                       const cnoid::BodyPtr& A_robot,//input
                                       const cnoid::BodyPtr& B_robot,//input
                                       const Eigen::MatrixXd& A_AMJ, //[joint root]の順. comまわり input
                                       const Eigen::MatrixXd& B_AMJ, //[joint root]の順. comまわり input
                                       std::unordered_map<cnoid::LinkPtr,int>& jacobianColMap, //input
                                       Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian//output
                                       );

}

#endif
