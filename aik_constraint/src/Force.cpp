#include <aik_constraint/Force.h>
#include <cnoid/MeshGenerator>

namespace aik_constraint{
  const std::vector<cnoid::SgNodePtr>& Force::getDrawOnObjects() {
    if(!this->lines_){
      this->lines_ = new cnoid::SgLineSet;
      this->lines_->setLineWidth(5.0);
      this->lines_->getOrCreateColors()->resize(2);
      this->lines_->getOrCreateColors()->at(0) = cnoid::Vector3f(204/255.0,51/255.0,51/255.0); // rvizと同じ
      this->lines_->getOrCreateColors()->at(1) = cnoid::Vector3f(204/255.0,204/255.0,51/255.0); // rvizと同じ
      // 0, F, N
      this->lines_->getOrCreateVertices()->resize(3);
      this->lines_->colorIndices().resize(0);
      this->lines_->addLine(0,1); this->lines_->colorIndices().push_back(0); this->lines_->colorIndices().push_back(0);
      this->lines_->addLine(0,2); this->lines_->colorIndices().push_back(1); this->lines_->colorIndices().push_back(1);

      this->drawOnObjects_ = std::vector<cnoid::SgNodePtr>{this->lines_};
    }

    cnoid::Vector6 f_local = this->S_ * this->F_;
    cnoid::Isometry3 pose = this->A_link_ ? this->A_link_->T() * this->A_localpos_ : this->A_localpos_;

    this->lines_->getOrCreateVertices()->at(0) = pose.translation().cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(1) = (pose * (f_local.head<3>() * 0.01)).cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(2) = (pose * (f_local.tail<3>() * 0.01)).cast<cnoid::Vector3f::Scalar>();

    return this->drawOnObjects_;

  }


  void Force::setFACE(std::shared_ptr<Force>& force) {
    force->S() = Eigen::SparseMatrix<double,Eigen::RowMajor>(6,6);
    for(int i=0;i<6;i++) force->S().insert(i,i) = 1.0;
    force->F() = cnoid::VectorX::Zero(6);
  }

  void Force::setSPAN(std::shared_ptr<Force>& force, double mu) {
    force->S() = Eigen::SparseMatrix<double,Eigen::RowMajor>(6,4);
    double norm = std::sqrt(std::pow(1.0,2) + std::pow(mu,2) + std::pow(mu,2));
    double nz = 1.0 / norm;
    double nxy = mu / norm;
    force->S().insert(0,0) = 1/nxy; force->S().insert(0,1) =-1/nxy; force->S().insert(0,2) =-1/nxy; force->S().insert(0,3) = 1/nxy;
    force->S().insert(1,0) = 1/nxy; force->S().insert(1,1) = 1/nxy; force->S().insert(1,2) =-1/nxy; force->S().insert(1,3) =-1/nxy;
    force->S().insert(2,0) = 1/nz; force->S().insert(2,1) = 1/nz; force->S().insert(2,2) = 1/nz; force->S().insert(2,3) = 1/nz;
    force->F() = cnoid::VectorX::Zero(4);
  }
};
