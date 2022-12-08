#include <aik_constraint/PositionConstraint.h>
#include <aik_constraint/Jacobian.h>

namespace aik_constraint{

  void PositionConstraint::update (const std::vector<cnoid::LinkPtr>& joints) {
    const cnoid::Position A_parent_pose = (this->A_link_) ? this->A_link_->T() : cnoid::Position::Identity(); // world frame
    const cnoid::Position B_parent_pose = (this->B_link_) ? this->B_link_->T() : cnoid::Position::Identity(); // world frame
    const cnoid::Position& A_pos = A_parent_pose * this->A_localpos_; // world frame
    const cnoid::Position& B_pos = B_parent_pose * this->B_localpos_; // world frame
    cnoid::Vector6 A_vel = cnoid::Vector6::Zero(); // world frame
    if(this->A_link_){
      A_vel.head<3>() += this->A_link_->v();
      A_vel.head<3>() += this->A_link_->w().cross(A_parent_pose.linear() * this->A_localpos_.translation());
      A_vel.tail<3>() += this->A_link_->w();
    }
    A_vel.head<3>() += A_parent_pose.linear() * this->A_localvel_.head<3>();
    A_vel.tail<3>() += A_parent_pose.linear() * this->A_localvel_.tail<3>();
    cnoid::Vector6 B_vel = cnoid::Vector6::Zero(); // world frame
    if(this->B_link_){
      B_vel.head<3>() += this->B_link_->v();
      B_vel.head<3>() += this->B_link_->w().cross(B_parent_pose.linear() * this->B_localpos_.translation());
      B_vel.tail<3>() += this->B_link_->w();
    }
    B_vel.head<3>() += B_parent_pose.linear() * this->B_localvel_.head<3>();
    B_vel.tail<3>() += B_parent_pose.linear() * this->B_localvel_.tail<3>();
    cnoid::Vector6 A_acc = cnoid::Vector6::Zero(); // world frame
    if(this->A_link_){
      A_acc.head<3>() += this->A_link_->dv();
      A_acc.head<3>() += this->A_link_->dw().cross(A_parent_pose.linear() * this->A_localpos_.translation()) + this->A_link_->w().cross(A_parent_pose.linear() * this->A_localvel_.head<3>());
      A_acc.tail<3>() += this->A_link_->dw();
    }
    cnoid::Vector6 B_acc = cnoid::Vector6::Zero(); // world frame
    if(this->B_link_){
      B_acc.head<3>() += this->B_link_->dv();
      B_acc.head<3>() += this->B_link_->dw().cross(B_parent_pose.linear() * this->B_localpos_.translation()) + this->B_link_->w().cross(B_parent_pose.linear() * this->B_localvel_.head<3>());
      B_acc.tail<3>() += this->B_link_->dw();
    }

    cnoid::Vector6 pos_error; // world frame. A - B
    {
      cnoid::AngleAxis angleAxis = cnoid::AngleAxis(A_pos.linear() * B_pos.linear().transpose());
      pos_error << A_pos.translation() - B_pos.translation() , angleAxis.angle()*angleAxis.axis();
    }
    cnoid::Vector6 vel_error = A_vel - B_vel; // world frame. A - B

    cnoid::Matrix3d eval_R = (this->eval_link_) ? this->eval_link_->R() * this->eval_localR_ : this->eval_localR_;
    cnoid::Vector6 pos_error_eval; // eval frame. A - B
    pos_error_eval.head<3>() = (eval_R.transpose() * pos_error.head<3>()).eval();
    pos_error_eval.tail<3>() = (eval_R.transpose() * pos_error.tail<3>()).eval();
    cnoid::Vector6 vel_error_eval; // eval frame. A - B
    vel_error_eval.head<3>() = (eval_R.transpose() * vel_error.head<3>()).eval();
    vel_error_eval.tail<3>() = (eval_R.transpose() * vel_error.tail<3>()).eval();
    cnoid::Vector6 target_acc = cnoid::Vector6::Zero();
    target_acc += this->ref_acc_;
    target_acc += this->clamp(cnoid::Vector6(this->pgain_.cwiseProduct(pos_error_eval)), this->maxAccByPosError_);
    target_acc += this->clamp(cnoid::Vector6(this->dgain_.cwiseProduct(vel_error_eval)), this->maxAccByVelError_);
    target_acc += - (B_acc - A_acc); // eval frame. B - A
    target_acc = this->clamp(target_acc, this->maxAcc_);

    {
      // B-Aの目標加速度を計算し、this->eq_に入れる
      if(this->eq_.rows()!=(this->weight_.array() > 0.0).count()) this->eq_ = Eigen::VectorXd((this->weight_.array() > 0.0).count());
      for(size_t i=0, idx=0; i<6; i++){
        if(this->weight_[i]>0.0) {
          this->eq_[idx] = target_acc[i] * this->weight_[i];
          idx++;
        }
      }
    }

    {
      // 行列の初期化. 前回とcol形状が変わっていないなら再利用
      if(!this->isJointsSame(joints,this->jacobian_joints_)
         || this->A_link_ != this->jacobian_A_link_
         || this->B_link_ != this->jacobian_B_link_){
        this->jacobian_joints_ = joints;
        this->jacobian_A_link_ = this->A_link_;
        this->jacobian_B_link_ = this->B_link_;

        aik_constraint::calc6DofJacobianShape(this->jacobian_joints_,//input
                                              this->jacobian_A_link_,//input
                                              this->jacobian_B_link_,//input
                                              true,//input
                                              this->jacobian_full_,
                                              this->jacobianColMap_,
                                              this->path_A_joints_,
                                              this->path_B_joints_,
                                              this->path_BA_joints_,
                                              this->path_BA_joints_numUpwardConnections_
                                              );
      }

      aik_constraint::calc6DofJacobianCoef(this->jacobian_joints_,//input
                                           this->jacobian_A_link_,//input
                                           this->A_localpos_,//input
                                           this->jacobian_B_link_,//input
                                           this->B_localpos_,//input
                                           this->jacobianColMap_,//input
                                           this->path_A_joints_,//input
                                           this->path_B_joints_,//input
                                           this->path_BA_joints_,//input
                                           this->path_BA_joints_numUpwardConnections_,//input
                                           true,//input
                                           this->jacobian_full_
                                           );

      cnoid::Matrix3d eval_R_dense = (this->eval_link_) ? this->eval_link_->R() * this->eval_localR_ : this->eval_localR_;
      Eigen::SparseMatrix<double,Eigen::RowMajor> eval_R(3,3);
      for(int i=0;i<3;i++) for(int j=0;j<3;j++) eval_R.insert(i,j) = eval_R_dense(i,j);
      this->jacobian_full_local_.resize(this->jacobian_full_.rows(), this->jacobian_full_.cols());
      this->jacobian_full_local_.topRows<3>() = eval_R.transpose() * this->jacobian_full_.topRows<3>();
      this->jacobian_full_local_.bottomRows<3>() = eval_R.transpose() * this->jacobian_full_.bottomRows<3>();

      this->jacobian_.resize((this->weight_.array() > 0.0).count(),this->jacobian_full_local_.cols());
      for(size_t i=0, idx=0;i<6;i++){
        if(this->weight_[i]>0.0) {
          this->jacobian_.row(idx) = - this->weight_[i] * this->jacobian_full_local_.row(i); // マイナス: A-B をB-Aに変換
          idx++;
        }
      }
    }

    if(this->debugLevel_>=1){
      std::cerr << "PositionConstraint" << std::endl;
      std::cerr << "A_pos" << std::endl;
      std::cerr << A_pos.translation().transpose() << std::endl;
      std::cerr << A_pos.linear() << std::endl;
      std::cerr << "A_vel" << std::endl;
      std::cerr << A_vel.transpose() << std::endl;
      std::cerr << "B_pos" << std::endl;
      std::cerr << B_pos.translation().transpose() << std::endl;
      std::cerr << B_pos.linear() << std::endl;
      std::cerr << "B_vel" << std::endl;
      std::cerr << B_vel.transpose() << std::endl;
      std::cerr << "target_acc" << std::endl;
      std::cerr << target_acc.transpose() << std::endl;
      std::cerr << "eq" << std::endl;
      std::cerr << this->eq_.transpose() << std::endl;
      std::cerr << "jacobian" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
    }

    return;
  }

  const std::vector<cnoid::SgNodePtr>& PositionConstraint::getDrawOnObjects(){
    if(!this->lines_){
      this->lines_ = new cnoid::SgLineSet;
      this->lines_->setLineWidth(1.0);
      this->lines_->getOrCreateColors()->resize(4);
      this->lines_->getOrCreateColors()->at(0) = cnoid::Vector3f(1.0,1.0,1.0);
      this->lines_->getOrCreateColors()->at(1) = cnoid::Vector3f(1.0,0.0,0.0);
      this->lines_->getOrCreateColors()->at(2) = cnoid::Vector3f(0.0,1.0,0.0);
      this->lines_->getOrCreateColors()->at(3) = cnoid::Vector3f(0.0,0.0,1.0);
      // A, A_x, A_y, A_z, B, B_x, B_y, B_z
      this->lines_->getOrCreateVertices()->resize(8);
      this->lines_->colorIndices().resize(0);
      this->lines_->addLine(0,1); this->lines_->colorIndices().push_back(1); this->lines_->colorIndices().push_back(1);
      this->lines_->addLine(0,2); this->lines_->colorIndices().push_back(2); this->lines_->colorIndices().push_back(2);
      this->lines_->addLine(0,3); this->lines_->colorIndices().push_back(3); this->lines_->colorIndices().push_back(3);
      this->lines_->addLine(4,5); this->lines_->colorIndices().push_back(1); this->lines_->colorIndices().push_back(1);
      this->lines_->addLine(4,6); this->lines_->colorIndices().push_back(2); this->lines_->colorIndices().push_back(2);
      this->lines_->addLine(4,7); this->lines_->colorIndices().push_back(3); this->lines_->colorIndices().push_back(3);
      this->lines_->addLine(0,4); this->lines_->colorIndices().push_back(0); this->lines_->colorIndices().push_back(0);

      this->drawOnObjects_ = std::vector<cnoid::SgNodePtr>{this->lines_};
    }

    const cnoid::Position& A_pos = (this->A_link_) ? this->A_link_->T() * this->A_localpos_ : this->A_localpos_;
    const cnoid::Position& B_pos = (this->B_link_) ? this->B_link_->T() * this->B_localpos_ : this->B_localpos_;

    this->lines_->getOrCreateVertices()->at(0) = A_pos.translation().cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(1) = (A_pos * (0.05 * cnoid::Vector3::UnitX())).cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(2) = (A_pos * (0.05 * cnoid::Vector3::UnitY())).cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(3) = (A_pos * (0.05 * cnoid::Vector3::UnitZ())).cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(4) = B_pos.translation().cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(5) = (B_pos * (0.05 * cnoid::Vector3::UnitX())).cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(6) = (B_pos * (0.05 * cnoid::Vector3::UnitY())).cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(7) = (B_pos * (0.05 * cnoid::Vector3::UnitZ())).cast<cnoid::Vector3f::Scalar>();

    return this->drawOnObjects_;
  }
}
