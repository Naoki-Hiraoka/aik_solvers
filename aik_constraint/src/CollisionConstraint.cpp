#include <iostream>

#include <aik_constraint/CollisionConstraint.h>
#include <aik_constraint/Jacobian.h>
#include <cnoid/EigenUtil>

namespace aik_constraint{

  void CollisionConstraint::update (const std::vector<cnoid::LinkPtr>& joints) {

    double distance;
    bool solved = this->computeDistance(this->A_link_, this->B_link_,
                                        distance, this->currentDirection_, this->A_currentLocalp_,this->B_currentLocalp_);
    if(!solved) distance = 1e10;

    cnoid::Vector3 A_vel = cnoid::Vector3::Zero(); // world frame
    if(this->A_link_){
      A_vel += this->A_link_->v();
      A_vel += this->A_link_->w().cross(this->A_link_->R() * this->A_currentLocalp_);
    }
    cnoid::Vector3 B_vel = cnoid::Vector3::Zero(); // world frame
    if(this->B_link_){
      B_vel += this->B_link_->v();
      B_vel += this->B_link_->w().cross(this->B_link_->R() * this->B_currentLocalp_);
    }
    cnoid::Vector3 A_acc = cnoid::Vector3::Zero(); // world frame
    if(this->A_link_){
      A_acc.head<3>() += this->A_link_->dv();
      A_acc.head<3>() += this->A_link_->dw().cross(this->A_link_->R() * this->A_currentLocalp_);
      A_acc.tail<3>() += this->A_link_->dw();
    }
    cnoid::Vector3 B_acc = cnoid::Vector3::Zero(); // world frame
    if(this->B_link_){
      B_acc.head<3>() += this->B_link_->dv();
      B_acc.head<3>() += this->B_link_->dw().cross(this->B_link_->R() * this->B_currentLocalp_);
      B_acc.tail<3>() += this->B_link_->dw();
    }

    double d_distance = (A_vel - B_vel).dot(this->currentDirection_);
    double dd_distance = (A_acc - B_acc).dot(this->currentDirection_);

    double min_dd_distance = 0.0;
    min_dd_distance += std::min(this->pgain_ * (this->tolerance_ - distance), this->maxAccByPosError_);
    min_dd_distance += std::min(this->dgain_ * ( - d_distance), this->maxAccByVelError_);
    min_dd_distance -= dd_distance;
    min_dd_distance = std::min(min_dd_distance, this->maxAcc_);

    if(this->minIneq_.rows()!=1) this->minIneq_ = Eigen::VectorXd::Zero(1);
    if(this->maxIneq_.rows()!=1) this->maxIneq_ = Eigen::VectorXd::Zero(1);
    this->minIneq_[0] = min_dd_distance * this->weight_;
    this->maxIneq_[0] = 1e10;


    // calc jacobian
    // 行列の初期化. 前回とcol形状が変わっていないなら再利用
    if(!this->isJointsSame(joints,this->jacobianIneq_joints_)
       || this->A_link_ != this->jacobianIneq_A_link_
       || this->B_link_ != this->jacobianIneq_B_link_){
      this->jacobianIneq_joints_ = joints;
      this->jacobianIneq_A_link_ = this->A_link_;
      this->jacobianIneq_B_link_ = this->B_link_;

      aik_constraint::calc6DofJacobianShape(this->jacobianIneq_joints_,//input
                                            this->jacobianIneq_A_link_,//input
                                            this->jacobianIneq_B_link_,//input
                                            false,//input
                                            this->jacobianIneq_full_,
                                            this->jacobianIneqColMap_,
                                            this->path_A_joints_,
                                            this->path_B_joints_,
                                            this->path_BA_joints_,
                                            this->path_BA_joints_numUpwardConnections_
                                            );
    }

    cnoid::Position A_localpos = cnoid::Position::Identity();
    A_localpos.translation() = this->A_currentLocalp_;
    cnoid::Position B_localpos = cnoid::Position::Identity();
    B_localpos.translation() = this->B_currentLocalp_;
    aik_constraint::calc6DofJacobianCoef(this->jacobianIneq_joints_,//input
                                         this->jacobianIneq_A_link_,//input
                                         A_localpos,//input
                                         this->jacobianIneq_B_link_,//input
                                         B_localpos,//input
                                         this->jacobianIneqColMap_,//input
                                         this->path_A_joints_,//input
                                         this->path_B_joints_,//input
                                         this->path_BA_joints_,//input
                                         this->path_BA_joints_numUpwardConnections_,//input
                                         false,//input
                                         this->jacobianIneq_full_
                                         );

    Eigen::SparseMatrix<double,Eigen::RowMajor> dir(3,1);
    for(int i=0;i<3;i++) dir.insert(i,0) = this->currentDirection_[i];
    this->jacobianIneq_ = dir.transpose() * this->jacobianIneq_full_ * this->weight_;


    if(this->debugLevel_>=1){
      std::cerr << "CollisionConstraint " << this->A_link_->name() << " - " << this->B_link_->name() << std::endl;
      std::cerr << "distance: " << distance << std::endl;
      std::cerr << "d_distance: " << d_distance << std::endl;
      std::cerr << "minIneq" << std::endl;
      std::cerr << this->minIneq_ << std::endl;
      std::cerr << "jacobianIneq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;
  }

  const std::vector<cnoid::SgNodePtr>& CollisionConstraint::getDrawOnObjects(){
    if(!this->lines_){
      this->lines_ = new cnoid::SgLineSet;
      this->lines_->setLineWidth(1.0);
      this->lines_->getOrCreateColors()->resize(1);
      this->lines_->getOrCreateColors()->at(0) = cnoid::Vector3f(0.0,0.0,0.5);
      // A, B
      this->lines_->getOrCreateVertices()->resize(2);
      this->lines_->colorIndices().resize(0);
      this->lines_->addLine(0,1); this->lines_->colorIndices().push_back(0); this->lines_->colorIndices().push_back(0);

      this->drawOnObjects_ = std::vector<cnoid::SgNodePtr>{this->lines_};
    }

    const cnoid::Vector3& A_pos = (this->A_link_) ? this->A_link_->T() * this->A_currentLocalp_ : this->B_currentLocalp_;
    const cnoid::Vector3& B_pos = (this->B_link_) ? this->B_link_->T() * this->B_currentLocalp_ : this->B_currentLocalp_;

    this->lines_->getOrCreateVertices()->at(0) = A_pos.cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(1) = B_pos.cast<cnoid::Vector3f::Scalar>();

    return this->drawOnObjects_;
  }

}
