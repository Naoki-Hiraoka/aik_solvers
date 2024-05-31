#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <cnoid/src/Body/InverseDynamics.h>
#include <iostream>
#include <ros/package.h>

#include <acc_lp_solver/acc_lp_solver.h>
#include <aik_constraint/aik_constraint.h>

namespace acc_lp_solver_sample{
  void sample1(){
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
    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > constraints;

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

      {
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
        constraints.push_back(constraint);
      }
    }
    {
      // lleg
      std::shared_ptr<aik_constraint::Force> force = std::make_shared<aik_constraint::Force>();
      force->A_link() = robot->link("LLEG_ANKLE_R");
      force->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.045);
      force->B_link() = nullptr;
      aik_constraint::Force::setFACE(force);
      forces.push_back(force);

      {
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
        constraints.push_back(constraint);
      }
    }

    {
      // task: EOM
      std::shared_ptr<aik_constraint::EOMConstraint> constraint = std::make_shared<aik_constraint::EOMConstraint>();
      constraint->robot() = robot;
      constraints.push_back(constraint);
    }

    // target
    std::shared_ptr<aik_constraint::Force> force = std::make_shared<aik_constraint::Force>();
    force->A_link() = nullptr;
    force->A_localpos().translation() = robot->centerOfMass();
    force->B_link() = robot->rootLink();
    force->S().resize(6,1);
    force->S().insert(0,0) = 0.0;
    force->S().insert(1,0) = 1.0;
    force->S().insert(2,0) = 0.0;
    force->F() = cnoid::VectorX::Zero(1);
    forces.push_back(force);

    int debugLevel = 1; // 0 or 1 or 2
    std::vector<cnoid::LinkPtr> variables;
    for(size_t i=0;i<constraints.size();i++){
      constraints[i]->debugLevel() = debugLevel;//debug
    }

    acc_lp_solver::IKParam param;
    param.debugLevel = debugLevel;
    //param.lpTolerance = 1e-7;
    bool solved = acc_lp_solver::solveLP(force,
                                         variables,
                                         forces,
                                         constraints,
                                         param);
    if(!solved) std::cerr << "!solved" << std::endl;
    else std::cerr << "solved" << std::endl;

    // visualize
    std::vector<cnoid::SgNodePtr> markers;
    for(int j=0;j<constraints.size();j++){
      const std::vector<cnoid::SgNodePtr>& marker = constraints[j]->getDrawOnObjects();
      std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
    }
    for(int j=0;j<forces.size();j++){
      const std::vector<cnoid::SgNodePtr>& marker = forces[j]->getDrawOnObjects();
      std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
    }
    viewer.drawOn(markers);
    viewer.drawObjects();

    return;
  }
}
