#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <cnoid/src/Body/InverseDynamics.h>
#include <iostream>
#include <ros/package.h>

#include <prioritized_acc_inverse_kinematics_solver/PrioritizedAccInverseKinematicsSolver.h>
#include <prioritized_qp_osqp/prioritized_qp_osqp.h>
#include <aik_constraint/PositionConstraint.h>
#include <aik_constraint/JointAngleConstraint.h>

namespace prioritized_acc_inverse_kinematics_solver_sample{
  void sample4_4limb_move(){
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
    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > constraints1;
    {
      // task: rleg to target
      std::shared_ptr<aik_constraint::PositionConstraint> constraint = std::make_shared<aik_constraint::PositionConstraint>();
      constraint->A_link() = robot->link("RLEG_ANKLE_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.04);
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(0.0,-0.2,-0.0);
      constraints1.push_back(constraint);
    }
    {
      // task: lleg to target
      std::shared_ptr<aik_constraint::PositionConstraint> constraint = std::make_shared<aik_constraint::PositionConstraint>();
      constraint->A_link() = robot->link("LLEG_ANKLE_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.04);
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(0.0,0.2,0.0);
      constraints1.push_back(constraint);
    }

    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > constraints2;
    std::shared_ptr<aik_constraint::PositionConstraint> rarmConstraint;
    {
      // task: rarm to target. never reach
      std::shared_ptr<aik_constraint::PositionConstraint> constraint = std::make_shared<aik_constraint::PositionConstraint>();
      constraint->A_link() = robot->link("RARM_WRIST_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.02);
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(0.0,-0.2,0.8);
      constraint->B_localpos().linear() = cnoid::Matrix3(cnoid::AngleAxis(-1.5,cnoid::Vector3(0,1,0)));
      constraints2.push_back(constraint);
      rarmConstraint = constraint;
    }
    {
      // task: larm to target. rotation-axis nil
      std::shared_ptr<aik_constraint::PositionConstraint> constraint = std::make_shared<aik_constraint::PositionConstraint>();
      constraint->A_link() = robot->link("LARM_WRIST_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.02);
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(0.0,0.2,0.8);
      constraint->B_localpos().linear() = cnoid::Matrix3(cnoid::AngleAxis(-1.5,cnoid::Vector3(0,1,0)));
      for(size_t i=0;i<3;i++)constraint->weight()[3+i] = 0.0;
      constraints2.push_back(constraint);
    }

    {
      // task: joint angle to target
      for(int i=0;i<robot->numJoints();i++){
        std::shared_ptr<aik_constraint::JointAngleConstraint> constraint = std::make_shared<aik_constraint::JointAngleConstraint>();
        constraint->joint() = robot->joint(i);
        constraint->targetq() = reset_manip_pose[i];
        constraint->weight() = 0.03;
        constraints2.push_back(constraint);
      }
    }


    int debugLevel = 0; // 0 or 1
    std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
    std::vector<cnoid::LinkPtr> variables;
    variables.push_back(robot->rootLink());
    for(size_t i=0;i<robot->numJoints();i++){
      variables.push_back(robot->joint(i));
    }
    std::vector<std::vector<std::shared_ptr<aik_constraint::IKConstraint> > > constraints{constraints0,constraints1,constraints2};
    for(size_t i=0;i<constraints.size();i++){
      for(size_t j=0;j<constraints[i].size();j++){
        constraints[i][j]->debugLevel() = debugLevel;//debug
      }
    }


    // main loop
    double dt = 0.002;
    for(int i=0;i< 300 / dt;i++){

      // move goal
      rarmConstraint->B_localpos().translation() = cnoid::Vector3(1.0+0.6*std::sin(i*dt),-0.2,0.8);
      rarmConstraint->B_localvel().head<3>() = cnoid::Vector3(0.6*std::cos(i*dt),0,0);

      prioritized_acc_inverse_kinematics_solver::IKParam param;
      param.debugLevel = debugLevel;
      param.wn = 1e-6;
      bool solved = prioritized_acc_inverse_kinematics_solver::solveAIK(variables,
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
        viewer.drawOn(markers);
        viewer.drawObjects();
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

      // sleep
      //std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000 / 2)));
    }

    std::cout << "finished" << std::endl;
    return;
  }
}
