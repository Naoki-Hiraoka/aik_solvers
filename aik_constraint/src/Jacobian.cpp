#include <aik_constraint/Jacobian.h>

#include <cnoid/LinkPath>
#include <cnoid/Jacobian>

namespace aik_constraint {
  inline size_t getJointDOF(const cnoid::LinkPtr& joint) {
    if(joint->isRevoluteJoint() || joint->isPrismaticJoint()) return 1;
    else if(joint->isFreeJoint()) return 6;
    else return 0;
  }

  inline void pushBackTripletList(std::vector<Eigen::Triplet<double> >& tripletList, const cnoid::LinkPtr& joint, int idx, const bool& calcRotation = true){
    if(joint->isFreeJoint()){
      for(size_t d=0;d<3;d++){
        tripletList.push_back(Eigen::Triplet<double>(d,idx+d,1));
      }

      if(calcRotation){
        for(size_t d=3;d<6;d++){
          tripletList.push_back(Eigen::Triplet<double>(d,idx+d,1));
        }
      }

      //  0     p[2] -p[1]
      // -p[2]  0     p[0]
      //  p[1] -p[0]  0
      tripletList.push_back(Eigen::Triplet<double>(0,idx+4,1));
      tripletList.push_back(Eigen::Triplet<double>(0,idx+5,1));
      tripletList.push_back(Eigen::Triplet<double>(1,idx+3,1));
      tripletList.push_back(Eigen::Triplet<double>(1,idx+5,1));
      tripletList.push_back(Eigen::Triplet<double>(2,idx+3,1));
      tripletList.push_back(Eigen::Triplet<double>(2,idx+4,1));

    } else if(joint->isRotationalJoint()){
      tripletList.push_back(Eigen::Triplet<double>(0,idx,1));
      tripletList.push_back(Eigen::Triplet<double>(1,idx,1));
      tripletList.push_back(Eigen::Triplet<double>(2,idx,1));
      if(calcRotation){
        tripletList.push_back(Eigen::Triplet<double>(3,idx,1));
        tripletList.push_back(Eigen::Triplet<double>(4,idx,1));
        tripletList.push_back(Eigen::Triplet<double>(5,idx,1));
      }

    } else if(joint->isPrismaticJoint()){
      tripletList.push_back(Eigen::Triplet<double>(0,idx,1));
      tripletList.push_back(Eigen::Triplet<double>(1,idx,1));
      tripletList.push_back(Eigen::Triplet<double>(2,idx,1));

    }
  }

  inline void fillJacobian(Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian, const cnoid::Vector3& target_p, const cnoid::LinkPtr& joint, int idx, int sign, const bool& calcRotation = true){
    if(joint->isFreeJoint()){
      //root 6dof
      for(size_t j=0;j<3;j++){
        jacobian.coeffRef(j,idx+j) = sign;
      }

      if(calcRotation){
        for(size_t j=3;j<6;j++){
          jacobian.coeffRef(j,idx+j) = sign;
        }
      }

      cnoid::Vector3 dp = target_p - joint->p();
      //  0     p[2] -p[1]
      // -p[2]  0     p[0]
      //  p[1] -p[0]  0
      jacobian.coeffRef(0,idx+4)=sign*dp[2];
      jacobian.coeffRef(0,idx+5)=-sign*dp[1];
      jacobian.coeffRef(1,idx+3)=-sign*dp[2];
      jacobian.coeffRef(1,idx+5)=sign*dp[0];
      jacobian.coeffRef(2,idx+3)=sign*dp[1];
      jacobian.coeffRef(2,idx+4)=-sign*dp[0];

    } else if(joint->isRotationalJoint()){
      cnoid::Vector3 omega = joint->R() * joint->a();
      cnoid::Vector3 dp = omega.cross(target_p - joint->p());
      jacobian.coeffRef(0,idx)=sign*dp[0];
      jacobian.coeffRef(1,idx)=sign*dp[1];
      jacobian.coeffRef(2,idx)=sign*dp[2];
      if(calcRotation){
        jacobian.coeffRef(3,idx)=sign*omega[0];
        jacobian.coeffRef(4,idx)=sign*omega[1];
        jacobian.coeffRef(5,idx)=sign*omega[2];
      }

    } else if(joint->isPrismaticJoint()){
      cnoid::Vector3 dp = joint->R() * joint->d();
      jacobian.coeffRef(0,idx)=sign*dp[0];
      jacobian.coeffRef(1,idx)=sign*dp[1];
      jacobian.coeffRef(2,idx)=sign*dp[2];
    }
  }

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
                             ){
    jacobianColMap.clear();
    int num_variables = 0;
    for(size_t i=0;i<joints.size();i++){
      jacobianColMap[joints[i]] = num_variables;
      num_variables += getJointDOF(joints[i]);
    }

    std::vector<Eigen::Triplet<double> > tripletList;
    tripletList.reserve(100);//適当

    if(!A_link || !B_link || !(A_link->body() == B_link->body())){
      // A, Bが関節を共有しない. 別々に処理すれば良い
      for(size_t i=0;i<2;i++){//0:A_link, 1:B_link
        cnoid::LinkPtr target_link = i ? B_link : A_link;
        std::vector<cnoid::LinkPtr>& path_joints = i ? path_B_joints : path_A_joints;

        if(!target_link) continue;//world固定なので飛ばす

        path_joints.clear();
        cnoid::LinkPath path(target_link);
        for(size_t j=0;j<path.size();j++){
          path_joints.push_back(path[j]);
        }

        for(size_t j=0;j<path_joints.size();j++){
          cnoid::LinkPtr joint = path_joints[j];
          if(jacobianColMap.find(joint)==jacobianColMap.end()) continue;
          int idx = jacobianColMap[joint];
          pushBackTripletList(tripletList,joint,idx,calcRotation);
        }
      }
    } else { //if(!A_link || !B_link || !(A_link->body() == B_link->body()))
      //A,Bが関節を共有する. 一つのpathで考える
      path_BA_joints.clear();
      {
        cnoid::LinkPath path(B_link,A_link);
        size_t j=0;
        for(;!path.isDownward(j);j++) path_BA_joints.push_back(path[j]);
        path_BA_joints_numUpwardConnections = j;
        j++;
        for(;j<path.size();j++) path_BA_joints.push_back(path[j]);
      }

      for(size_t j=0;j<path_BA_joints.size();j++){
        cnoid::LinkPtr joint = path_BA_joints[j];
        if(jacobianColMap.find(joint)==jacobianColMap.end()) continue;
        int idx = jacobianColMap[joint];
        pushBackTripletList(tripletList,joint,idx,calcRotation);
      }
    }

    jacobian = Eigen::SparseMatrix<double,Eigen::RowMajor>(calcRotation?6:3,num_variables);
    jacobian.setFromTriplets(tripletList.begin(), tripletList.end());

  }

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
                            ) {
    if(!A_link || !B_link || !(A_link->body() == B_link->body())){
      // A, Bが関節を共有しない. 別々に処理すれば良い
      for(size_t i=0;i<2;i++){//0:A_link, 1:B_link
        int sign = i ? -1 : 1;
        cnoid::LinkPtr target_link = i ? B_link : A_link;
        const cnoid::Position& target_localpos = i ? B_localpos : A_localpos;
        const std::vector<cnoid::LinkPtr>& path_joints = i ? path_B_joints : path_A_joints;

        if(!target_link) continue;//world固定なので飛ばす

        const cnoid::Position target_position = target_link->T() * target_localpos;
        const cnoid::Vector3 target_p = target_position.translation();

        for(size_t j=0;j<path_joints.size();j++){
          cnoid::LinkPtr joint = path_joints[j];
          if(jacobianColMap.find(joint)==jacobianColMap.end()) continue;
          int idx = jacobianColMap[joint];
          fillJacobian(jacobian,target_p,joint,idx,sign,calcRotation);
        }
      }
    } else { //if(!A_link || !B_link || !(A_link->body() == B_link->body()))
      //A,Bが関節を共有する. 一つのpathで考える
      const cnoid::Vector3& target_p = A_link->T() * A_localpos.translation();
      for(size_t j=0;j<path_BA_joints.size();j++){
          cnoid::LinkPtr joint = path_BA_joints[j];
          if(jacobianColMap.find(joint)==jacobianColMap.end()) continue;
          int idx = jacobianColMap[joint];
          if(j<path_BA_joints_numUpwardConnections){
            fillJacobian(jacobian,target_p,joint,idx,-1,calcRotation);
          } else {
            fillJacobian(jacobian,target_p,joint,idx,1,calcRotation);
          }
      }
    }
  }

  void calcCMJacobianShape(const std::vector<cnoid::LinkPtr>& joints,
                           const cnoid::BodyPtr& A_robot,
                           const cnoid::BodyPtr& B_robot,
                           Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian,
                           std::unordered_map<cnoid::LinkPtr,int>& jacobianColMap){
    jacobianColMap.clear();
    int cols = 0;
    for(size_t i=0;i<joints.size();i++){
      jacobianColMap[joints[i]] = cols;
      cols += getJointDOF(joints[i]);
    }
    jacobian = Eigen::SparseMatrix<double,Eigen::RowMajor>(3,cols);

    std::vector<Eigen::Triplet<double> > tripletList;
    tripletList.reserve(100);//適当

    if(A_robot != B_robot){
      for(int i=0;i<2;i++){
        cnoid::BodyPtr robot = (i==0) ? A_robot : B_robot;
        if(!robot) continue;
        if(jacobianColMap.find(robot->rootLink()) != jacobianColMap.end()){
          if(robot->rootLink()->isFreeJoint()){
            int idx = jacobianColMap[robot->rootLink()];
            for(size_t row=0;row<3;row++){
              for(size_t j=0;j<getJointDOF(robot->rootLink());j++){
                tripletList.push_back(Eigen::Triplet<double>(row,idx+j,1));
              }
            }
          }
        }
        for(size_t j=0;j<robot->numJoints();j++){
          if(jacobianColMap.find(robot->joint(j)) != jacobianColMap.end()){
            int idx = jacobianColMap[robot->joint(j)];
            for(size_t row=0;row<3;row++){
              for(size_t k=0;k<getJointDOF(robot->joint(j));k++){
                tripletList.push_back(Eigen::Triplet<double>(row,idx+k,1));
              }
            }
          }
        }
      }
    }
    jacobian.setFromTriplets(tripletList.begin(), tripletList.end());
  }

  void calcCMJacobianCoef(const std::vector<cnoid::LinkPtr>& joints,//input
                          const cnoid::BodyPtr& A_robot,//input
                          const cnoid::BodyPtr& B_robot,//input
                          const Eigen::MatrixXd& A_CMJ, //[joint root]の順 input
                          const Eigen::MatrixXd& B_CMJ, //[joint root]の順 input
                          std::unordered_map<cnoid::LinkPtr,int>& jacobianColMap, //input
                          Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian//output
                          ) {
    if(A_robot != B_robot){
      for(size_t i=0;i<2; i++){
        cnoid::BodyPtr robot = (i==0) ? A_robot : B_robot;
        if(!robot) continue;
        int sign = (i==0) ? 1 : -1;

        const Eigen::MatrixXd& CMJ = (i==0) ? A_CMJ : B_CMJ; // [joint root]の順

        if(jacobianColMap.find(robot->rootLink()) != jacobianColMap.end()){
          if(robot->rootLink()->isFreeJoint()){
            int col_idx = jacobianColMap[robot->rootLink()];
            for(size_t j=0;j<3;j++){
              for(size_t k=0;k<getJointDOF(robot->rootLink());k++){
                jacobian.coeffRef(j,col_idx+k) = sign * CMJ(j,robot->numJoints()+k);
              }
            }
          }
        }
        for(size_t j=0;j<robot->numJoints();j++){
          if(jacobianColMap.find(robot->joint(j)) != jacobianColMap.end()){
            int col_idx = jacobianColMap[robot->joint(j)];
            for(size_t k=0;k<3;k++){
              for(size_t d=0;d<getJointDOF(robot->joint(j));d++){
                jacobian.coeffRef(k,col_idx+d) = sign * CMJ(k,j+d);
              }
            }
          }
        }
      }
    }

  }


  namespace cnoid18 {
    // choreonoidのrelease1.7の calcAngularMomentumJacobianにはバグがあり、開発版ではhttps://github.com/s-nakaoka/choreonoid/pull/234 で修正されている. 修正された版の関数(https://github.com/choreonoid/choreonoid/blob/master/src/Body/Jacobian.cpp )を使う
    inline cnoid::Matrix3d D(cnoid::Vector3d r)
    {
      cnoid::Matrix3d r_cross;
      r_cross <<
        0.0,  -r(2), r(1),
        r(2),    0.0,  -r(0),
        -r(1), r(0),    0.0;
      return r_cross.transpose() * r_cross;
    }

    struct SubMass
    {
      double m;
      cnoid::Vector3 mwc;
      cnoid::Matrix3d Iw;
      SubMass& operator+=(const SubMass& rhs){
        m += rhs.m;
        mwc += rhs.mwc;
        Iw += rhs.Iw;
        return *this;
      }
    };

    inline void calcSubMass(cnoid::Link* link, std::vector<SubMass>& subMasses, bool calcIw)
    {
      cnoid::Matrix3d R = link->R();
      SubMass& sub = subMasses[link->index()];
      sub.m = link->m();
      sub.mwc = link->m() * link->wc();

      for(cnoid::Link* child = link->child(); child; child = child->sibling()){
        calcSubMass(child, subMasses, calcIw);
        SubMass& childSub = subMasses[child->index()];
        sub.m += childSub.m;
        sub.mwc += childSub.mwc;
      }

      if(calcIw){
        if(sub.m != 0.0) sub.Iw = R * link->I() * R.transpose() + link->m() * D( link->wc() - sub.mwc/sub.m );
        else sub.Iw = R * link->I() * R.transpose();
        for(cnoid::Link* child = link->child(); child; child = child->sibling()){
          SubMass& childSub = subMasses[child->index()];
          if(sub.m != 0.0 && childSub.m != 0.0) sub.Iw += childSub.Iw + childSub.m * D( childSub.mwc/childSub.m - sub.mwc/sub.m );
          else sub.Iw += childSub.Iw;
        }
      }
    }
    void calcAngularMomentumJacobian(cnoid::Body* body, cnoid::Link* base, Eigen::MatrixXd& H)
    {

      // prepare subm, submwc

      const int nj = body->numJoints();
      std::vector<SubMass> subMasses(body->numLinks());
      cnoid::Link* rootLink = body->rootLink();
      std::vector<int> sgn(nj, 1);

      cnoid::MatrixXd M;
      calcCMJacobian( body, base, M );
      M.conservativeResize(3, nj);
      M *= body->mass();

      if(!base){
        calcSubMass(rootLink, subMasses, true);
        H.resize(3, nj + 6);

      } else {
        cnoid::JointPath path(rootLink, base);
        cnoid::Link* skip = path.joint(0);
        SubMass& sub = subMasses[skip->index()];
        sub.m = rootLink->m();
        sub.mwc = rootLink->m() * rootLink->wc();

        for(cnoid::Link* child = rootLink->child(); child; child = child->sibling()){
          if(child != skip){
            calcSubMass(child, subMasses, true);
            sub += subMasses[child->index()];
          }
        }

        // assuming there is no branch between base and root
        for(int i=1; i < path.numJoints(); i++){
          cnoid::Link* joint = path.joint(i);
          const cnoid::Link* parent = joint->parent();
          SubMass& sub = subMasses[joint->index()];
          sub.m = parent->m();
          sub.mwc = parent->m() * parent->wc();
          sub += subMasses[parent->index()];
        }

        H.resize(3, nj);

        for(int i=0; i < path.numJoints(); i++){
          sgn[path.joint(i)->jointId()] = -1;
        }
      }

      // compute Jacobian
      for(int i=0; i < nj; ++i){
        cnoid::Link* joint = body->joint(i);
        if(joint->isRotationalJoint()){
          const cnoid::Vector3 omega = sgn[joint->jointId()] * joint->R() * joint->a();
          const SubMass& sub = subMasses[joint->index()];
          const cnoid::Vector3 Mcol = M.col(joint->jointId());
          cnoid::Vector3 dp;
          if(sub.m != 0.0) dp = (sub.mwc/sub.m).cross(Mcol) + sub.Iw * omega;
          else dp = sub.Iw * omega;
          H.col(joint->jointId()) = dp;
        } else {
          //unsupported jointType
        }
      }

      if(!base){
        const int c = nj;
        H.block(0, c, 3, 3).setZero();
        H.block(0, c+3, 3, 3) = subMasses[rootLink->index()].Iw;

        cnoid::Vector3 cm = body->calcCenterOfMass();
        cnoid::Matrix3d cm_cross;
        cm_cross <<
          0.0,  -cm(2), cm(1),
          cm(2),    0.0,  -cm(0),
          -cm(1), cm(0),    0.0;
        H.block(0,0,3,c) -= cm_cross * M;
      }
    }
  }

  void calcAngularMomentumJacobianShape(const std::vector<cnoid::LinkPtr>& joints,
                                        const cnoid::BodyPtr& A_robot,
                                        const cnoid::BodyPtr& B_robot,
                                        Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian,
                                        std::unordered_map<cnoid::LinkPtr,int>& jacobianColMap){
    jacobianColMap.clear();
    int cols = 0;
    for(size_t i=0;i<joints.size();i++){
      jacobianColMap[joints[i]] = cols;
      cols += getJointDOF(joints[i]);
    }
    jacobian = Eigen::SparseMatrix<double,Eigen::RowMajor>(3,cols);

    std::vector<Eigen::Triplet<double> > tripletList;
    tripletList.reserve(100);//適当

    if(A_robot != B_robot){
      for(int i=0;i<2;i++){
        cnoid::BodyPtr robot = (i==0) ? A_robot : B_robot;
        if(!robot) continue;
        if(jacobianColMap.find(robot->rootLink()) != jacobianColMap.end()){
          if(robot->rootLink()->isFreeJoint()){
            int idx = jacobianColMap[robot->rootLink()];
            for(size_t row=0;row<3;row++){
              for(size_t j=3;j<getJointDOF(robot->rootLink());j++){//並進は無視
                tripletList.push_back(Eigen::Triplet<double>(row,idx+j,1));
              }
            }
          }
        }
        for(size_t j=0;j<robot->numJoints();j++){
          if(jacobianColMap.find(robot->joint(j)) != jacobianColMap.end()){
            int idx = jacobianColMap[robot->joint(j)];
            for(size_t row=0;row<3;row++){
              for(size_t k=0;k<getJointDOF(robot->joint(j));k++){
                tripletList.push_back(Eigen::Triplet<double>(row,idx+k,1));
              }
            }
          }
        }
      }
    }
    jacobian.setFromTriplets(tripletList.begin(), tripletList.end());
  }

  void calcAngularMomentumJacobianCoef(const std::vector<cnoid::LinkPtr>& joints,//input
                                       const cnoid::BodyPtr& A_robot,//input
                                       const cnoid::BodyPtr& B_robot,//input
                                       const Eigen::MatrixXd& A_AMJ, //[joint root]の順. comまわり input
                                       const Eigen::MatrixXd& B_AMJ, //[joint root]の順. comまわり input
                                       std::unordered_map<cnoid::LinkPtr,int>& jacobianColMap, //input
                                       Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian//output
                                       ) {
    if(A_robot != B_robot){
      for(size_t i=0;i<2; i++){
        cnoid::BodyPtr robot = (i==0) ? A_robot : B_robot;
        if(!robot) continue;
        int sign = (i==0) ? 1 : -1;

        const Eigen::MatrixXd& AMJ = (i==0) ? A_AMJ : B_AMJ; // [joint root]の順. comまわり

        if(jacobianColMap.find(robot->rootLink()) != jacobianColMap.end()){
          if(robot->rootLink()->isFreeJoint()){
            int col_idx = jacobianColMap[robot->rootLink()];
            for(size_t j=0;j<3;j++){
              for(size_t k=3;k<getJointDOF(robot->rootLink());k++){ // 並進は無視
                jacobian.coeffRef(j,col_idx+k) = sign * AMJ(j,robot->numJoints()+k);
              }
            }
          }
        }
        for(size_t j=0;j<robot->numJoints();j++){
          if(jacobianColMap.find(robot->joint(j)) != jacobianColMap.end()){
            int col_idx = jacobianColMap[robot->joint(j)];
            for(size_t k=0;k<3;k++){
              for(size_t d=0;d<getJointDOF(robot->joint(j));d++){
                jacobian.coeffRef(k,col_idx+d) = sign * AMJ(k,j+d);
              }
            }
          }
        }
      }
    }
  }

}




