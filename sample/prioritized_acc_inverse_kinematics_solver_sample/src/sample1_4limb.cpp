#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <iostream>
#include <ros/package.h>
#include "sample1_4limb.h"

#include <prioritized_acc_inverse_kinematics_solver/PrioritizedAccInverseKinematicsSolver.h>
#include <prioritized_qp_osqp/prioritized_qp_osqp.h>
#include <aik_constraint/PositionConstraint.h>

void sample1_4limb(){
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
  robot->calcForwardKinematics(true,true);
  robot->calcCenterOfMass();


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
  {
    // task: rarm to target.
    std::shared_ptr<aik_constraint::PositionConstraint> constraint = std::make_shared<aik_constraint::PositionConstraint>();
    constraint->A_link() = robot->link("RARM_WRIST_R");
    constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.02);
    constraint->B_link() = nullptr;
    constraint->B_localpos().translation() = cnoid::Vector3(0.6,-0.2,0.8);
    constraint->B_localpos().linear() = cnoid::Matrix3(cnoid::AngleAxis(-1.5,cnoid::Vector3(0,1,0)));
    constraints2.push_back(constraint);
  }
  {
    // task: larm to target. rotation-axis nil
    std::shared_ptr<aik_constraint::PositionConstraint> constraint = std::make_shared<aik_constraint::PositionConstraint>();
    constraint->A_link() = robot->link("LARM_WRIST_R");
    constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.02);
    constraint->B_link() = nullptr;
    constraint->B_localpos().translation() = cnoid::Vector3(0.3,0.2,0.8);
    constraint->B_localpos().linear() = cnoid::Matrix3(cnoid::AngleAxis(-1.5,cnoid::Vector3(0,1,0)));
    for(size_t i=0;i<3;i++)constraint->weight()[3+i] = 0.0;
    constraints2.push_back(constraint);
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
  for(int i=0;i< 10 / dt;i++){
    prioritized_acc_inverse_kinematics_solver::IKParam param;
    param.debugLevel = debugLevel;
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
    robot->calcForwardKinematics(true, true);
    robot->calcCenterOfMass();

    // sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000 / 2)));
  }

  return;
}
