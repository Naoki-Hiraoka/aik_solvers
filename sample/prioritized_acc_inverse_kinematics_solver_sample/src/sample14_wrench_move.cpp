#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <cnoid/Jacobian>
#include <cnoid/src/Body/InverseDynamics.h>
#include <iostream>
#include <ros/package.h>

#include <prioritized_acc_inverse_kinematics_solver/PrioritizedAccInverseKinematicsSolver.h>
#include <prioritized_qp_osqp/prioritized_qp_osqp.h>
#include <aik_constraint/aik_constraint.h>

namespace prioritized_acc_inverse_kinematics_solver_sample{
  namespace footguidedcontroller{
    template <typename T>
    class LinearTrajectory
    {
    private:
      T a, start, goal;
      double time;
    public:
      LinearTrajectory(const T& _start, const T& _goal, const double _t)
        : start(_start), goal(_goal) {
        time = std::max(_t, 0.0);
        if(time==0) a = _start*0.0;
        else a = (_goal - _start) / time;
      };
      const T& getSlope() const { return a; };
      const T& getStart() const { return start; };
      const T& getGoal() const { return goal; };
      double getTime() const { return time; };
    };

    /*
      J = \int^T_0 (u - u^r)^2 dt
      \dot x = w ( x - u - l)
      x(0) = x0
      x(T) = u^r(T) + l
      u^r = std::vector<LinearTrajectory>
    */
    // ur_のサイズは1以上でなければならない. ur_にtime=0の要素があっても、その要素のstartとgoalが同じなら破綻しない. ただし、ur_のtimeの和が0だと破綻する
    template <typename T> T calcFootGuidedControl(const double& w, const T& l, const T& x0, const std::vector<LinearTrajectory<T> >& ur_) {
      const int n = ur_.size();

      std::vector<LinearTrajectory<T> > ur;
      ur.reserve(n + 2);
      ur.push_back(LinearTrajectory<T>(x0-l, x0-l, 0.0)); // j=0
      std::copy(ur_.begin(), ur_.end(), std::back_inserter(ur)); // j=1 ~ j=n
      ur.push_back(LinearTrajectory<T>(ur_.back().getGoal(),ur_.back().getGoal(), 0.0)); // j=n+1

      T u = x0*0.0;
      double Tj = 0.0;
      for(int j=0;j<=n;j++){
        Tj += ur.at(j).getTime();
        u += exp(- w * Tj) * (ur.at(j).getGoal() - ur.at(j+1).getStart() + (ur.at(j).getSlope() - ur.at(j+1).getSlope()) / w);
      }

      if((1 - exp(-2 * w * Tj)) == 0.0) { // ゼロ除算チェック
        std::cerr << "[calcFootGuidedControl] (1 - exp(-2 * w * Tj))==0 !" << std::endl;
        return ur[1].getStart();
      }

      return ur[1].getStart() + 2 / (1 - exp(-2 * w * Tj)) * u;
    };

    template <typename T> void updateState(const double& w, const T& l, const T& c, const T& dc, const T& u, double m, double dt,
                                           T& o_c, T& o_dc, T& o_ddc, T& o_f/*uから受ける力*/) {
      o_c = c + dc * dt;
      o_dc = dc + w * w * (c - u - l) * dt;
      o_ddc = w * w * (c - u - l);
      o_f = m * w * w * (c - u);
      return;
    };

  };

  void sample14_wrench_move(){
    // setup robot
    cnoid::BodyLoader bodyLoader;
    cnoid::BodyPtr robot = bodyLoader.load(ros::package::getPath("choreonoid") + "/share/model/SR1/SR1.body");
    // reset pose
    robot->rootLink()->p() = cnoid::Vector3(0,0,0.7);
    robot->rootLink()->v().setZero();
    robot->rootLink()->dv().setZero();
    robot->rootLink()->R() = cnoid::Matrix3::Identity();
    robot->rootLink()->w().setZero();
    robot->rootLink()->dw().setZero();
    std::vector<double> reset_manip_pose{
      0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// rleg
        0.523599, 0.0, 0.0, -1.74533, 0.15708, -0.113446, 0.637045,// rarm
        0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// lleg
        0.523599, 0.0, 0.0, -1.74533, -0.15708, -0.113446, -0.637045,// larm
        0.0, 0.0, 0.0};

    for(int j=0; j < robot->numJoints(); ++j){
      robot->joint(j)->q() = reset_manip_pose[j];
      robot->joint(j)->dq() = 0.0;
      robot->joint(j)->ddq() = 0.0;
    }
    for(int l=0;l<robot->numLinks();l++) robot->link(l)->F_ext().setZero();
    robot->calcForwardKinematics(true,true);
    robot->calcCenterOfMass();
    cnoid::Vector6 F_o = cnoid::calcInverseDynamics(robot->rootLink()); // world frame origin
    robot->rootLink()->F_ext().head<3>() = F_o.head<3>(); // rootLink origin
    robot->rootLink()->F_ext().tail<3>() = F_o.tail<3>() + (-robot->rootLink()->p()).cross(F_o.head<3>()); // rootLink origin


    // setup viewer
    choreonoid_viewer::Viewer viewer;
    viewer.objects(robot);

    // setup task
    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > constraints0;

    // setup Force
    std::vector<std::shared_ptr<aik_constraint::Force> > forces;

    {
      // rleg
      std::shared_ptr<aik_constraint::Force> force = std::make_shared<aik_constraint::Force>();
      force->A_link() = robot->link("RLEG_ANKLE_R");
      force->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.045);
      force->B_link() = nullptr;
      aik_constraint::Force::setFACE(force);
      forces.push_back(force);

      std::shared_ptr<aik_constraint::ForceConstraint> constraint = std::make_shared<aik_constraint::ForceConstraint>();
      constraint->force() = force;
      constraint->dl() = Eigen::VectorXd::Zero(11);
      constraint->C().resize(11,6);
      constraint->du() = 1e10 * Eigen::VectorXd::Ones(11);
      constraint->C().insert(0,2) = 1.0; constraint->du()[0] = 2000.0;
      constraint->C().insert(1,0) = 1.0; constraint->C().insert(1,2) = 0.2;
      constraint->C().insert(2,0) = -1.0; constraint->C().insert(2,2) = 0.2;
      constraint->C().insert(3,1) = 1.0; constraint->C().insert(3,2) = 0.2;
      constraint->C().insert(4,1) = -1.0; constraint->C().insert(4,2) = 0.2;
      constraint->C().insert(5,2) = 0.05; constraint->C().insert(5,3) = 1.0;
      constraint->C().insert(6,2) = 0.05; constraint->C().insert(6,3) = -1.0;
      constraint->C().insert(7,2) = 0.12; constraint->C().insert(7,4) = 1.0;
      constraint->C().insert(8,2) = 0.09; constraint->C().insert(8,4) = -1.0;
      constraint->C().insert(9,2) = 0.005; constraint->C().insert(9,5) = 1.0;
      constraint->C().insert(10,2) = 0.005; constraint->C().insert(10,5) = -1.0;
      constraints0.push_back(constraint);

    }
    {
      // lleg
      std::shared_ptr<aik_constraint::Force> force = std::make_shared<aik_constraint::Force>();
      force->A_link() = robot->link("LLEG_ANKLE_R");
      force->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.045);
      force->B_link() = nullptr;
      aik_constraint::Force::setFACE(force);
      forces.push_back(force);

      std::shared_ptr<aik_constraint::ForceConstraint> constraint = std::make_shared<aik_constraint::ForceConstraint>();
      constraint->force() = force;
      constraint->dl() = Eigen::VectorXd::Zero(11);
      constraint->C().resize(11,6);
      constraint->du() = 1e10 * Eigen::VectorXd::Ones(11);
      constraint->C().insert(0,2) = 1.0; constraint->du()[0] = 2000.0;
      constraint->C().insert(1,0) = 1.0; constraint->C().insert(1,2) = 0.2;
      constraint->C().insert(2,0) = -1.0; constraint->C().insert(2,2) = 0.2;
      constraint->C().insert(3,1) = 1.0; constraint->C().insert(3,2) = 0.2;
      constraint->C().insert(4,1) = -1.0; constraint->C().insert(4,2) = 0.2;
      constraint->C().insert(5,2) = 0.05; constraint->C().insert(5,3) = 1.0;
      constraint->C().insert(6,2) = 0.05; constraint->C().insert(6,3) = -1.0;
      constraint->C().insert(7,2) = 0.12; constraint->C().insert(7,4) = 1.0;
      constraint->C().insert(8,2) = 0.09; constraint->C().insert(8,4) = -1.0;
      constraint->C().insert(9,2) = 0.005; constraint->C().insert(9,5) = 1.0;
      constraint->C().insert(10,2) = 0.005; constraint->C().insert(10,5) = -1.0;
      constraints0.push_back(constraint);

    }

    {
      // task: joint angle limit
      for(int i=0;i<robot->numJoints();i++){
        std::shared_ptr<aik_constraint::JointLimitConstraint> constraint = std::make_shared<aik_constraint::JointLimitConstraint>();
        constraint->joint() = robot->joint(i);
        constraint->dgain() = 100;
        constraint->maxAccByVelError() = 20;
        constraints0.push_back(constraint);
      }
    }

    {
      // task: EOM
      std::shared_ptr<aik_constraint::EOMConstraint> constraint = std::make_shared<aik_constraint::EOMConstraint>();
      constraint->robot() = robot;
      constraints0.push_back(constraint);
    }

    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > constraints1;
    {
      // task: rleg to target
      std::shared_ptr<aik_constraint::PositionConstraint> constraint = std::make_shared<aik_constraint::PositionConstraint>();
      constraint->A_link() = robot->link("RLEG_ANKLE_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.04);
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(0.0,-0.1,0.0);
      constraints1.push_back(constraint);
    }
    {
      // task: lleg to target
      std::shared_ptr<aik_constraint::PositionConstraint> constraint = std::make_shared<aik_constraint::PositionConstraint>();
      constraint->A_link() = robot->link("LLEG_ANKLE_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.04);
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(0.0,0.1,0.0);
      constraints1.push_back(constraint);
    }

    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > constraints2;
    std::shared_ptr<aik_constraint::COMConstraint> comconstraint;
    {
      // task: COM to target
      std::shared_ptr<aik_constraint::COMConstraint> constraint = std::make_shared<aik_constraint::COMConstraint>();
      constraint->A_robot() = robot;
      constraint->B_localp() = cnoid::Vector3(0.0,0.0,0.7);
      constraint->weight() << 1.0, 1.0, 0.3;
      constraints2.push_back(constraint);
      comconstraint = constraint;
    }

    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > constraints3;
    {
      // task: joint angle to target
      for(int i=0;i<robot->numJoints();i++){
        std::shared_ptr<aik_constraint::JointAngleConstraint> constraint = std::make_shared<aik_constraint::JointAngleConstraint>();
        constraint->joint() = robot->joint(i);
        constraint->targetq() = reset_manip_pose[i];
        constraint->weight() = 1.0;
        constraints3.push_back(constraint);
      }
    }
    {
      // task: angular momentum to target
      std::shared_ptr<aik_constraint::AngularMomentumConstraint> constraint = std::make_shared<aik_constraint::AngularMomentumConstraint>();
      constraint->robot() = robot;
      constraint->weight() = 1.0 * cnoid::Vector3::Ones();
      constraints3.push_back(constraint);
    }

    int debugLevel = 1; // 0 or 1 or 2
    std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
    std::vector<cnoid::LinkPtr> variables;
    variables.push_back(robot->rootLink());
    for(size_t i=0;i<robot->numJoints();i++){
      variables.push_back(robot->joint(i));
    }
    std::vector<std::vector<std::shared_ptr<aik_constraint::IKConstraint> > > constraints{constraints0,constraints1,constraints2,constraints3};
    for(size_t i=0;i<constraints.size();i++){
      for(size_t j=0;j<constraints[i].size();j++){
        constraints[i][j]->debugLevel() = debugLevel;//debug
      }
    }

    // main loop
    double dt = 0.002;
    for(int i=0;i< 300 / dt;i++){

      if(i>1000){
        Eigen::MatrixXd CMJ;
        cnoid::calcCMJacobian(robot,nullptr,CMJ); // [joint root]の順
        cnoid::VectorX dq(robot->numJoints()+6);
        for(int i=0;i<robot->numJoints();i++) dq[i] = robot->joint(i)->dq();
        dq.segment<3>(robot->numJoints()) = robot->rootLink()->v();
        dq.tail<3>() = robot->rootLink()->w();
        cnoid::Vector3 c_v = CMJ * dq;
        double w = std::sqrt(9.8/0.7);
        cnoid::Vector3 cp = robot->centerOfMass() + c_v / w;

        int t = i % 2000;
        std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> > traj;
        double l = 0.10;
        double f = 250;
        if(t<1*f){
          traj.emplace_back(cnoid::Vector3(0.0,l*t/f,0.7),cnoid::Vector3(0.0,l,0.7),(1*f-t)*dt);
          traj.emplace_back(cnoid::Vector3(0.0,l,0.7),cnoid::Vector3(0.0,l,0.7),2*f*dt);
        }else if(t<3*f){
          traj.emplace_back(cnoid::Vector3(0.0,l,0.7),cnoid::Vector3(0.0,l,0.7),(3*f-t)*dt);
          traj.emplace_back(cnoid::Vector3(0.0,l,0.7),cnoid::Vector3(0.0,-l,0.7),2*f*dt);
        }else if(t<5*f){
          traj.emplace_back(cnoid::Vector3(0.0,l*(4*f-t)/f,0.7),cnoid::Vector3(0.0,-l,0.7),(5*f-t)*dt);
          traj.emplace_back(cnoid::Vector3(0.0,-l,0.7),cnoid::Vector3(0.0,-l,0.7),2*f*dt);
        }else if(t<7*f){
          traj.emplace_back(cnoid::Vector3(0.0,-l,0.7),cnoid::Vector3(0.0,-l,0.7),(7*f-t)*dt);
          traj.emplace_back(cnoid::Vector3(0.0,-l,0.7),cnoid::Vector3(0.0,l,0.7),2*f*dt);
        }else{
          traj.emplace_back(cnoid::Vector3(0.0,-l*(8*f-t)/f,0.7),cnoid::Vector3(0.0,l,0.7),(9*f-t)*dt);
          traj.emplace_back(cnoid::Vector3(0.0,l,0.7),cnoid::Vector3(0.0,l,0.7),2*f*dt);
        }
        comconstraint->pgain().setZero();
        comconstraint->dgain().setZero();
        cnoid::Vector3 vrp = footguidedcontroller::calcFootGuidedControl<cnoid::Vector3>(w,
                                                                                         cnoid::Vector3::Zero(),
                                                                                         cp,
                                                                                         traj);
        vrp[1] = std::min(0.14,std::max(-0.14,vrp[1]));
        comconstraint->ref_acc() = std::pow(w,2) * (robot->centerOfMass() - vrp);
        std::cerr << robot->centerOfMass()[1] << " " << cp[1] << " " << vrp[1] << " " << traj[0].getStart()[1] << " " << comconstraint->ref_acc()[1] << std::endl;
      }

      prioritized_acc_inverse_kinematics_solver::IKParam param;
      param.debugLevel = debugLevel;
      param.ddqWeight = 1e-3;
      param.forceWeight = 1e-12;
      bool solved = prioritized_acc_inverse_kinematics_solver::solveAIK(variables,
                                                                        forces,
                                                                        constraints,
                                                                        tasks,
                                                                        param);
      if(!solved) break;

      // visualize
      if( i % 50 == 0){
        std::vector<cnoid::SgNodePtr> markers;
        for(int j=0;j<constraints.size();j++){
          for(int k=0;k<constraints[j].size(); k++){
            const std::vector<cnoid::SgNodePtr>& marker = constraints[j][k]->getDrawOnObjects();
            std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
          }
        }
        for(int j=0;j<forces.size();j++){
          const std::vector<cnoid::SgNodePtr>& marker = forces[j]->getDrawOnObjects();
          std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
        }
        viewer.drawOn(markers);
        viewer.drawObjects();
      }

      {
        Eigen::MatrixXd CMJ;
        cnoid::calcCMJacobian(robot,nullptr,CMJ); // [joint root]の順
        cnoid::VectorX ddq(robot->numJoints()+6);
        for(int i=0;i<robot->numJoints();i++) ddq[i] = robot->joint(i)->ddq();
        ddq.segment<3>(robot->numJoints()) = robot->rootLink()->dv();
        ddq.tail<3>() = robot->rootLink()->dw();
        cnoid::Vector3 c_v = CMJ * ddq;
        std::cerr << c_v[1] << std::endl;
        std::cerr << forces[0]->F().transpose() << " *** " << forces[1]->F().transpose()<< std::endl;
      }

      // update state
      robot->rootLink()->p() += robot->rootLink()->v() * dt;
      robot->rootLink()->v() += robot->rootLink()->dv() * dt;
      robot->rootLink()->dv().setZero();
      if(robot->rootLink()->w().norm() != 0){
        robot->rootLink()->R() = cnoid::Matrix3(cnoid::AngleAxis(robot->rootLink()->w().norm() * dt, cnoid::Vector3(robot->rootLink()->w().normalized())) * cnoid::AngleAxis(robot->rootLink()->R()));
      }
      robot->rootLink()->w() += robot->rootLink()->dw() * dt;
      robot->rootLink()->dw().setZero();
      for(int j=0;j<robot->numJoints();j++){
        robot->joint(j)->q() += robot->joint(j)->dq() * dt;
        robot->joint(j)->dq() += robot->joint(j)->ddq() * dt;
        robot->joint(j)->ddq() = 0.0; // ddqは毎回0に戻し、Fは戻さない
      }
      for(int l=0;l<robot->numLinks();l++) robot->link(l)->F_ext().setZero();
      robot->calcForwardKinematics(true, true);
      robot->calcCenterOfMass();
      cnoid::Vector6 F_o = cnoid::calcInverseDynamics(robot->rootLink()); // world frame origin
      robot->rootLink()->F_ext().head<3>() = F_o.head<3>(); // rootLink origin
      robot->rootLink()->F_ext().tail<3>() = F_o.tail<3>() + (-robot->rootLink()->p()).cross(F_o.head<3>()); // rootLink origin

      for(int i=0;i<forces.size();i++){
        forces[i]->F().setZero();
      }

      // sleep
      std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000 / 2)));
    }

    std::cout << "finished" << std::endl;
    return;
  }
}
