#include <aik_constraint/AngularMomentumConstraint.h>
#include <cnoid/Jacobian>
#include <aik_constraint/Jacobian.h>

namespace aik_constraint{
  void AngularMomentumConstraint::update (const std::vector<cnoid::LinkPtr>& joints) {
    if(!this->robot_) {
      this->eq_.resize(0);
      this->jacobian_.resize(0,0);
      this->jacobian_robot_ = nullptr;
      this->jacobian_joints_.resize(0);
      return;
    }

    Eigen::MatrixXd AMJ;
    aik_constraint::cnoid18::calcAngularMomentumJacobian(this->robot_,nullptr,AMJ); // [joint root]の順. comまわり

    cnoid::Matrix3 I = AMJ.block<3,3>(0,this->robot_->numJoints()+3); //world系. comまわり
    cnoid::Matrix3 I_evalR = this->eval_R_.transpose() * I * this->eval_R_; //eval_R系
    Eigen::SparseMatrix<double,Eigen::RowMajor> I_evalR_inv(3,3); // eval_R系. この値でscaleする
    for(int i=0;i<3;i++) I_evalR_inv.insert(i,i) = 1.0/I_evalR(i,i);

    cnoid::Vector3 Iw = cnoid::Vector3::Zero(); // world frame. comまわり
    {
      cnoid::VectorX dq(this->robot_->numJoints()+6);
      for(int i=0;i<this->robot_->numJoints();i++) dq[i] = this->robot_->joint(i)->dq();
      dq.segment<3>(this->robot_->numJoints()) = this->robot_->rootLink()->v();
      dq.tail<3>() = this->robot_->rootLink()->w();
      Iw = AMJ * dq;
    }
    cnoid::Vector3 Idw = this->robot_->rootLink()->F_ext().tail<3>() + (this->robot_->rootLink()->p() - this->robot_->centerOfMass()).cross(this->robot_->rootLink()->F_ext().head<3>()); // world frame. comまわり

    cnoid::Vector3 Iw_error = this->target_Iw_ - Iw; // world frame. comまわり
    cnoid::Vector3 Idw_error = this->ref_Idw_ - Idw; // world frame. comまわり

    cnoid::Vector3 Iw_error_eval = this->eval_R_.transpose() * Iw_error; // eval frame. comまわり
    cnoid::Vector3 Idw_error_eval = this->eval_R_.transpose() * Idw_error; // eval frame. comまわり

    cnoid::Vector3 Iw_error_eval_scaled = I_evalR_inv * Iw_error_eval; // eval frame. comまわり
    cnoid::Vector3 Idw_error_eval_scaled = I_evalR_inv * Idw_error_eval; // eval frame. comまわり

    cnoid::Vector3 target_Idw_eval_scaled = cnoid::Vector3::Zero(); // eval frame. comまわり
    target_Idw_eval_scaled += this->clamp(cnoid::Vector3(this->dgain_.cwiseProduct(Iw_error_eval_scaled)), this->maxAccByVelError_);;
    target_Idw_eval_scaled += Idw_error_eval_scaled;
    target_Idw_eval_scaled = this->clamp(target_Idw_eval_scaled, this->maxAcc_);

    if(this->eq_.rows() != 3) this->eq_ = Eigen::VectorXd(3);
    for(size_t i=0, idx=0; i<3; i++){
      if(this->weight_[i]>0.0) {
        this->eq_[idx] = target_Idw_eval_scaled[i] * this->weight_[i];
        idx++;
      }
    }


    // calc jacobian
    // 行列の初期化. 前回とcol形状が変わっていないなら再利用
    if(!this->isJointsSame(joints,this->jacobian_joints_)
       || this->robot_ != this->jacobian_robot_){
      this->jacobian_joints_ = joints;
      this->jacobian_robot_ = this->robot_;

      aik_constraint::calcAngularMomentumJacobianShape(this->jacobian_joints_,
                                                       this->jacobian_robot_,
                                                       nullptr,
                                                       this->jacobian_full_,
                                                       this->jacobianColMap_);
    }

    aik_constraint::calcAngularMomentumJacobianCoef(this->jacobian_joints_,
                                                    this->jacobian_robot_,
                                                    nullptr,
                                                    AMJ,
                                                    AMJ,// not used
                                                    this->jacobianColMap_,
                                                    this->jacobian_full_);

    Eigen::SparseMatrix<double,Eigen::RowMajor> eval_R(3,3);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) eval_R.insert(i,j) = this->eval_R_(i,j);

    this->jacobian_full_local_ = I_evalR_inv * eval_R.transpose() * this->jacobian_full_;
    this->jacobian_.resize((this->weight_.array() > 0.0).count(),this->jacobian_full_local_.cols());
    for(size_t i=0, idx=0;i<3;i++){
      if(this->weight_[i]>0.0) {
        this->jacobian_.row(idx) = this->weight_[i] * this->jacobian_full_local_.row(i);
        idx++;
      }
    }

    if(this->debugLevel_>=1){
      std::cerr << "AngularMomentumConstraint" << std::endl;
      std::cerr << "Iw target_Iw" << std::endl;
      std::cerr << Iw.transpose() << "  " << this->target_Iw_.transpose() << std::endl;
      std::cerr << "I_evalR" << std::endl;
      std::cerr << I_evalR << std::endl;
      std::cerr << "target_Idw_eval_scaled" << std::endl;
      std::cerr << target_Idw_eval_scaled.transpose() << std::endl;
      std::cerr << "eq" << std::endl;
      std::cerr << this->eq_.transpose() << std::endl;
      std::cerr << "jacobian" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
    }

    return;
  }
}
