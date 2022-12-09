#include <aik_constraint/COMConstraint.h>
#include <aik_constraint/Jacobian.h>
#include <cnoid/Jacobian>

namespace aik_constraint{
  void COMConstraint::update (const std::vector<cnoid::LinkPtr>& joints) {

    Eigen::MatrixXd A_CMJ;
    if(this->A_robot_) cnoid::calcCMJacobian(this->A_robot_,nullptr,A_CMJ); // [joint root]の順
    Eigen::MatrixXd B_CMJ;
    if(this->B_robot_) cnoid::calcCMJacobian(this->B_robot_,nullptr,B_CMJ); // [joint root]の順

    cnoid::Vector3 A_p = this->A_robot_ ? this->A_robot_->centerOfMass() + this->A_localp_ : this->A_localp_; // world frame
    cnoid::Vector3 B_p = this->B_robot_ ? this->B_robot_->centerOfMass() + this->B_localp_ : this->B_localp_; // world frame
    cnoid::Vector3 A_v = cnoid::Vector3::Zero(); // world frame
    if(this->A_robot_) {
      cnoid::VectorX dq(this->A_robot_->numJoints()+6);
      for(int i=0;i<this->A_robot_->numJoints();i++) dq[i] = this->A_robot_->joint(i)->dq();
      for(int i=0;i<3;i++) {
        dq[this->A_robot_->numJoints()+i] = this->A_robot_->rootLink()->v()[i];
        dq[this->A_robot_->numJoints()+3+i] = this->A_robot_->rootLink()->w()[i];
      }
      A_v = A_CMJ * dq;
    }
    A_v += this->A_localv_;
    cnoid::Vector3 B_v = cnoid::Vector3::Zero(); // world frame
    if(this->B_robot_) {
      cnoid::VectorX dq(this->B_robot_->numJoints()+6);
      for(int i=0;i<this->B_robot_->numJoints();i++) dq[i] = this->B_robot_->joint(i)->dq();
      for(int i=0;i<3;i++) {
        dq[this->B_robot_->numJoints()+i] = this->B_robot_->rootLink()->v()[i];
        dq[this->B_robot_->numJoints()+3+i] = this->B_robot_->rootLink()->w()[i];
      }
      B_v = B_CMJ * dq;
    }
    B_v += this->B_localv_;
    cnoid::Vector3 A_a = cnoid::Vector3::Zero(); // world frame
    if(this->A_robot_ && this->A_robot_->mass()>0.0) A_a = this->A_robot_->rootLink()->F_ext().head<3>() / this->A_robot_->mass();
    cnoid::Vector3 B_a = cnoid::Vector3::Zero(); // world frame
    if(this->B_robot_ && this->B_robot_->mass()>0.0) B_a = this->B_robot_->rootLink()->F_ext().head<3>() / this->B_robot_->mass();

    // A - B
    cnoid::Vector3 pos_error = A_p - B_p; // world frame A - B
    cnoid::Vector3 vel_error = A_v - B_v; // world frame A - B

    cnoid::Vector3 pos_error_eval = this->eval_R_.transpose() * pos_error; // eval frame A - B
    cnoid::Vector3 vel_error_eval = this->eval_R_.transpose() * vel_error; // eval frame A - B

    cnoid::Vector3 target_acc = cnoid::Vector3::Zero();  // eval frame A - B
    target_acc += this->ref_acc_;
    target_acc -= this->clamp(cnoid::Vector3(this->pgain_.cwiseProduct(pos_error_eval)), this->maxAccByPosError_);
    target_acc -= this->clamp(cnoid::Vector3(this->dgain_.cwiseProduct(vel_error_eval)), this->maxAccByVelError_);
    target_acc -= A_a - B_a;
    target_acc = this->clamp(target_acc, this->maxAcc_);


    // A-Bの目標加速度を計算し、this->eq_に入れる
    if(this->eq_.rows()!=(this->weight_.array() > 0.0).count()) this->eq_ = Eigen::VectorXd((this->weight_.array() > 0.0).count());
    for(size_t i=0, idx=0; i<3; i++){
      if(this->weight_[i]>0.0) {
        this->eq_[idx] = target_acc[i] * this->weight_[i];
        idx++;
      }
    }

    // jacobianの計算
    // 行列の初期化. 前回とcol形状が変わっていないなら再利用
    if(!this->isJointsSame(joints,this->jacobian_joints_)
       || this->A_robot_ != this->jacobian_A_robot_
       || this->B_robot_ != this->jacobian_B_robot_){
      this->jacobian_joints_ = joints;
      this->jacobian_A_robot_ = this->A_robot_;
      this->jacobian_B_robot_ = this->B_robot_;

      aik_constraint::calcCMJacobianShape(this->jacobian_joints_,
                              this->jacobian_A_robot_,
                              this->jacobian_B_robot_,
                              this->jacobian_full_,
                              this->jacobianColMap_);
    }

    aik_constraint::calcCMJacobianCoef(this->jacobian_joints_,
                           this->jacobian_A_robot_,
                           this->jacobian_B_robot_,
                           A_CMJ,
                           B_CMJ,
                           this->jacobianColMap_,
                           this->jacobian_full_);

    Eigen::SparseMatrix<double,Eigen::RowMajor> eval_R(3,3);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) eval_R.insert(i,j) = this->eval_R_(i,j);
    this->jacobian_full_local_= eval_R.transpose() * this->jacobian_full_;

    this->jacobian_.resize((this->weight_.array() > 0.0).count(),this->jacobian_full_local_.cols());
    for(size_t i=0, idx=0;i<3;i++){
      if(this->weight_[i]>0.0) {
        this->jacobian_.row(idx) = this->weight_[i] * this->jacobian_full_local_.row(i);
        idx++;
      }
    }


    if(this->debugLevel_>=1){
      std::cerr << "COMConstraint" << std::endl;
      std::cerr << "A pos "<<A_p.transpose() << std::endl;
      std::cerr << "B pos "<<B_p.transpose() << std::endl;
      std::cerr << "A vel "<<A_v.transpose() << std::endl;
      std::cerr << "B vel "<<B_v.transpose() << std::endl;
      std::cerr << "target_acc "<<target_acc.transpose() << std::endl;
      std::cerr << "eq" << std::endl;
      std::cerr << this->eq_.transpose() << std::endl;
      std::cerr << "jacobian" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
    }

    return;
  }
}
