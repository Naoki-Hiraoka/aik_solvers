#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <cnoid/src/Body/InverseDynamics.h>
#include <iostream>
#include <ros/package.h>

#include <prioritized_acc_inverse_kinematics_solver/PrioritizedAccInverseKinematicsSolver.h>
#include <prioritized_qp_osqp/prioritized_qp_osqp.h>
#include <aik_constraint/aik_constraint.h>

namespace prioritized_acc_inverse_kinematics_solver_sample{
  void sample15_jaxon_limit(){
    // setup robot
    cnoid::BodyLoader bodyLoader;
    cnoid::BodyPtr robot = bodyLoader.load(ros::package::getPath("jvrc_models")+"/JAXON_JVRC/JAXON_JVRCmain_hrpsys.wrl");
    // reset pose
    robot->rootLink()->p() = cnoid::Vector3(0,0,1.0);
    robot->rootLink()->v().setZero();
    robot->rootLink()->dv().setZero();
    robot->rootLink()->R() = cnoid::Matrix3::Identity();
    robot->rootLink()->w().setZero();
    robot->rootLink()->dw().setZero();
    std::vector<double> reset_manip_pose{
      0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0,// rleg
        0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0,// lleg
        0.0, 0.0, 0.0, // torso
        0.0, 0.0, // head
        0.0, 0.959931, -0.349066, -0.261799, -1.74533, -0.436332, 0.0, -0.785398,// rarm
        0.0, 0.959931, 0.349066, 0.261799, -1.74533, 0.436332, 0.0, -0.785398,// larm
        0.0, 0.0 , 0.0, 0.0 // hand
        };

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
      // task: joint angle limit
      for(int i=0;i<robot->numJoints();i++){
        std::shared_ptr<aik_constraint::JointLimitConstraint> constraint = std::make_shared<aik_constraint::JointLimitConstraint>();
        constraint->joint() = robot->joint(i);
        constraint->dgain() = 100;
        constraints0.push_back(constraint);
      }
    }

    {
      // rleg
      std::shared_ptr<aik_constraint::Force> force = std::make_shared<aik_constraint::Force>();
      force->A_link() = robot->link("RLEG_JOINT5");
      force->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.1);
      force->B_link() = nullptr;
      aik_constraint::Force::setFACE(force);
      forces.push_back(force);

      std::shared_ptr<aik_constraint::ForceConstraint> constraint = std::make_shared<aik_constraint::ForceConstraint>();
      constraint->force() = force;
      constraint->dl() = Eigen::VectorXd::Zero(11);
      constraint->C().resize(11,6);
      constraint->du() = 1e10 * Eigen::VectorXd::Ones(11);
      constraint->dl()[0] = 50.0; constraint->C().insert(0,2) = 1.0; constraint->du()[0] = 2000.0;
      constraint->C().insert(1,0) = 1.0; constraint->C().insert(1,2) = 0.2;
      constraint->C().insert(2,0) = -1.0; constraint->C().insert(2,2) = 0.2;
      constraint->C().insert(3,1) = 1.0; constraint->C().insert(3,2) = 0.2;
      constraint->C().insert(4,1) = -1.0; constraint->C().insert(4,2) = 0.2;
      constraint->C().insert(5,2) = 0.05; constraint->C().insert(5,3) = 1.0;
      constraint->C().insert(6,2) = 0.05; constraint->C().insert(6,3) = -1.0;
      constraint->C().insert(7,2) = 0.1; constraint->C().insert(7,4) = 1.0;
      constraint->C().insert(8,2) = 0.1; constraint->C().insert(8,4) = -1.0;
      constraint->C().insert(9,2) = 0.005; constraint->C().insert(9,5) = 1.0;
      constraint->C().insert(10,2) = 0.005; constraint->C().insert(10,5) = -1.0;
      constraints0.push_back(constraint);

    }
    {
      // lleg
      std::shared_ptr<aik_constraint::Force> force = std::make_shared<aik_constraint::Force>();
      force->A_link() = robot->link("LLEG_JOINT5");
      force->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.1);
      force->B_link() = nullptr;
      aik_constraint::Force::setFACE(force);
      forces.push_back(force);

      std::shared_ptr<aik_constraint::ForceConstraint> constraint = std::make_shared<aik_constraint::ForceConstraint>();
      constraint->force() = force;
      constraint->dl() = Eigen::VectorXd::Zero(11);
      constraint->C().resize(11,6);
      constraint->du() = 1e10 * Eigen::VectorXd::Ones(11);
      constraint->dl()[0] = 50.0; constraint->C().insert(0,2) = 1.0; constraint->du()[0] = 2000.0;
      constraint->C().insert(1,0) = 1.0; constraint->C().insert(1,2) = 0.2;
      constraint->C().insert(2,0) = -1.0; constraint->C().insert(2,2) = 0.2;
      constraint->C().insert(3,1) = 1.0; constraint->C().insert(3,2) = 0.2;
      constraint->C().insert(4,1) = -1.0; constraint->C().insert(4,2) = 0.2;
      constraint->C().insert(5,2) = 0.05; constraint->C().insert(5,3) = 1.0;
      constraint->C().insert(6,2) = 0.05; constraint->C().insert(6,3) = -1.0;
      constraint->C().insert(7,2) = 0.1; constraint->C().insert(7,4) = 1.0;
      constraint->C().insert(8,2) = 0.1; constraint->C().insert(8,4) = -1.0;
      constraint->C().insert(9,2) = 0.005; constraint->C().insert(9,5) = 1.0;
      constraint->C().insert(10,2) = 0.005; constraint->C().insert(10,5) = -1.0;
      constraints0.push_back(constraint);
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
      constraint->A_link() = robot->link("RLEG_JOINT5");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.1);
      constraint->B_link() = nullptr;
      constraint->B_localpos() = constraint->A_link()->T() * constraint->A_localpos();
      constraints1.push_back(constraint);
    }
    {
      // task: lleg to target
      std::shared_ptr<aik_constraint::PositionConstraint> constraint = std::make_shared<aik_constraint::PositionConstraint>();
      constraint->A_link() = robot->link("LLEG_JOINT5");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.1);
      constraint->B_link() = nullptr;
      constraint->B_localpos() = constraint->A_link()->T() * constraint->A_localpos();
      constraints1.push_back(constraint);
    }
    {
      // task: COM to target
      std::shared_ptr<aik_constraint::COMConstraint> constraint = std::make_shared<aik_constraint::COMConstraint>();
      constraint->A_robot() = robot;
      constraint->B_localp() = cnoid::Vector3(0.0,0.0,1.0);
      constraint->maxAcc() << 0.5, 0.5, 0.5;
      constraints1.push_back(constraint);
    }

    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > constraints2;
    {
      // task: rarm to target.
      std::shared_ptr<aik_constraint::PositionConstraint> constraint = std::make_shared<aik_constraint::PositionConstraint>();
      constraint->A_link() = robot->link("RARM_JOINT7");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.22);
      constraint->A_localpos().linear() = cnoid::AngleAxisd(M_PI/2,cnoid::Vector3::UnitY()).toRotationMatrix();
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(0.4,-0.4,1.0);
      constraint->pgain() << 200.0, 200.0, 200.0, 100.0, 100.0, 100.0;
      constraint->dgain() << 30.0, 30.0, 30.0, 20.0, 20.0, 20.0;
      constraints2.push_back(constraint);
    }
    {
      // task: larm to target.
      std::shared_ptr<aik_constraint::PositionConstraint> constraint = std::make_shared<aik_constraint::PositionConstraint>();
      constraint->A_link() = robot->link("LARM_JOINT7");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.22);
      constraint->A_localpos().linear() = cnoid::AngleAxisd(M_PI/2,cnoid::Vector3::UnitY()).toRotationMatrix();
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(0.4,0.4,1.0);
      constraint->pgain() << 200.0, 200.0, 200.0, 100.0, 100.0, 100.0;
      constraint->dgain() << 30.0, 30.0, 30.0, 20.0, 20.0, 20.0;
      constraints2.push_back(constraint);
    }
    {
      // task: root to target.
      std::shared_ptr<aik_constraint::PositionConstraint> constraint = std::make_shared<aik_constraint::PositionConstraint>();
      constraint->A_link() = robot->rootLink();
      constraint->B_link() = nullptr;
      constraint->weight() << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0;
      constraint->pgain() << 200.0, 200.0, 200.0, 100.0, 100.0, 100.0;
      constraint->dgain() << 30.0, 30.0, 30.0, 20.0, 20.0, 20.0;
      constraints2.push_back(constraint);
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
      constraint->weight() = 0.1 * cnoid::Vector3::Ones();
      constraints3.push_back(constraint);
    }


    int debugLevel = 0; // 0 or 1 or 2
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
      prioritized_acc_inverse_kinematics_solver::IKParam param;
      param.debugLevel = debugLevel;
      param.ddqWeight = 1e-3;//1e-6;
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
        getchar();
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
        robot->joint(j)->ddq() = 0.0;
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
      //std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000 / 2)));
    }

    std::cout << "finished" << std::endl;
    return;
  }
}
