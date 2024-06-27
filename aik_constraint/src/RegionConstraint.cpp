#include <aik_constraint/RegionConstraint.h>
#include <aik_constraint/Jacobian.h>

namespace aik_constraint{

  void RegionConstraint::update (const std::vector<cnoid::LinkPtr>& joints, const std::vector<std::shared_ptr<Force> >& forces) {
    const cnoid::Isometry3 A_parent_pose = (this->A_link_) ? this->A_link_->T() : cnoid::Isometry3::Identity(); // world frame
    const cnoid::Isometry3 B_parent_pose = (this->B_link_) ? this->B_link_->T() : cnoid::Isometry3::Identity(); // world frame
    const cnoid::Vector3& A_pos = A_parent_pose * this->A_localpos_; // world frame
    const cnoid::Vector3& B_pos = B_parent_pose * this->B_localpos_; // world frame
    cnoid::Vector3 A_vel = cnoid::Vector3::Zero(); // world frame
    if(this->A_link_){
      A_vel += this->A_link_->v();
      A_vel += this->A_link_->w().cross(A_parent_pose.linear() * this->A_localpos_);
    }
    A_vel += A_parent_pose.linear() * this->A_localvel_;
    cnoid::Vector3 B_vel = cnoid::Vector3::Zero(); // world frame
    if(this->B_link_){
      B_vel += this->B_link_->v();
      B_vel += this->B_link_->w().cross(B_parent_pose.linear() * this->B_localpos_);
    }
    B_vel.head<3>() += B_parent_pose.linear() * this->B_localvel_;
    cnoid::Vector3 A_acc = cnoid::Vector3::Zero(); // world frame
    if(this->A_link_){
      A_acc += this->A_link_->dv();
      A_acc += this->A_link_->dw().cross(A_parent_pose.linear() * this->A_localpos_) + this->A_link_->w().cross(A_parent_pose.linear() * this->A_localvel_);
    }
    cnoid::Vector3 B_acc = cnoid::Vector3::Zero(); // world frame
    if(this->B_link_){
      B_acc += this->B_link_->dv();
      B_acc += this->B_link_->dw().cross(B_parent_pose.linear() * this->B_localpos_) + this->B_link_->w().cross(B_parent_pose.linear() * this->B_localvel_);
    }

    cnoid::Vector3 pos_error = A_pos - B_pos; // world frame. A - B
    cnoid::Vector3 vel_error = A_vel - B_vel; // world frame. A - B
    cnoid::Vector3 acc_error = A_acc - B_acc; // world frame. A - B

    cnoid::Matrix3d eval_R = (this->eval_link_) ? this->eval_link_->R() * this->eval_localR_ : this->eval_localR_;
    cnoid::Vector3 pos_error_eval = eval_R.transpose() * pos_error; // eval frame. A - B
    cnoid::Vector3 vel_error_eval = eval_R.transpose() * vel_error; // eval frame. A - B
    cnoid::Vector3 acc_error_eval = eval_R.transpose() * acc_error; // eval frame. A - B
    cnoid::Vector3 target_acc = this->ref_acc_; // eval frame. A - B

    cnoid::Vector3 current_error = cnoid::Vector3::Zero(); // eval frame. A - B
    current_error += pos_error_eval * this->pgain_;
    current_error += vel_error_eval * this->dgain_;
    current_error += acc_error_eval;
    current_error -= this->ref_acc_;

    if(this->weight_ <= 0.0 ||
       this->C_.rows() == 0) {
      this->minIneq_.resize(0);
      this->maxIneq_.resize(0);
    }else{
      if(this->C_.rows() != this->dl_.rows() ||
         this->C_.rows() != this->du_.rows() ||
         (this->C_.rows() != 0 && this->C_.cols() != 3)) {
        std::cerr << __FUNCTION__ << "dimension mismatch" << std::endl;
        return;
      }
      Eigen::VectorXd current = this->C_ * current_error;
      this->minIneq_ = (this->dl_-current).array().min(this->maxAcc_) * this->weight_;
      this->maxIneq_ = (this->du_-current).array().max(-this->maxAcc_) * this->weight_;
    }

    {
      // 行列の初期化. 前回とcol形状が変わっていないなら再利用
      if(!this->isJointsSame(joints,this->jacobian_joints_)
         || !this->isForcesSame(forces,this->jacobian_forces_)
         || this->A_link_ != this->jacobian_A_link_
         || this->B_link_ != this->jacobian_B_link_){
        this->jacobian_joints_ = joints;
        this->jacobian_forces_ = forces;
        this->jacobian_A_link_ = this->A_link_;
        this->jacobian_B_link_ = this->B_link_;

        aik_constraint::calc6DofJacobianShape(this->jacobian_joints_,//input
                                              this->jacobian_forces_,//input
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
                                           this->jacobian_forces_,//input
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
      this->jacobian_full_local_ = eval_R.transpose() * this->jacobian_full_.topRows<3>();

      if(this->weight_ <= 0.0 ||
         this->C_.rows() == 0) {
        this->jacobianIneq_.resize(0,this->jacobian_full_local_.cols());
      }else{
        this->jacobianIneq_ = this->weight_ * this->C_ * this->jacobian_full_local_;
      }
    }

    if(this->debugLevel_>=2){
      std::cerr << "RegionConstraint " << (this->A_link_ ? this->A_link_->name() : std::string("")) << " : " << (this->B_link_ ? this->B_link_->name() : std::string("")) << std::endl;
      std::cerr << "A_pos" << std::endl;
      std::cerr << A_pos.transpose() << std::endl;
      std::cerr << "A_vel" << std::endl;
      std::cerr << A_vel.transpose() << std::endl;
      std::cerr << "B_pos" << std::endl;
      std::cerr << B_pos.transpose() << std::endl;
      std::cerr << "B_vel" << std::endl;
      std::cerr << B_vel.transpose() << std::endl;
      std::cerr << "current_error" << std::endl;
      std::cerr << target_acc.transpose() << std::endl;
      std::cerr << "minIneq" << std::endl;
      std::cerr << this->minIneq_.transpose() << std::endl;
      std::cerr << "maxneq" << std::endl;
      std::cerr << this->maxIneq_.transpose() << std::endl;
      std::cerr << "jacobianIneq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;
  }

  const std::vector<cnoid::SgNodePtr>& RegionConstraint::getDrawOnObjects(){
    if(!this->lines_){
      this->lines_ = new cnoid::SgLineSet;
      this->lines_->setLineWidth(1.0);
      this->lines_->getOrCreateColors()->resize(1);
      this->lines_->getOrCreateColors()->at(0) = cnoid::Vector3f(0.3,0.3,0.5);
      // A, B
      this->lines_->getOrCreateVertices()->resize(2);
      this->lines_->colorIndices().resize(0);
      this->lines_->addLine(0,1); this->lines_->colorIndices().push_back(0); this->lines_->colorIndices().push_back(0);

      this->drawOnObjects_ = std::vector<cnoid::SgNodePtr>{this->lines_};
    }

    const cnoid::Vector3& A_pos = (this->A_link_) ? this->A_link_->T() * this->A_localpos_ : this->A_localpos_;
    const cnoid::Vector3& B_pos = (this->B_link_) ? this->B_link_->T() * this->B_localpos_ : this->B_localpos_;

    this->lines_->getOrCreateVertices()->at(0) = A_pos.cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(1) = B_pos.cast<cnoid::Vector3f::Scalar>();

    return this->drawOnObjects_;
  }
}
