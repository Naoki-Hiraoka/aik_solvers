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
  void sample12_wrench(){
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
      std::vector<cnoid::Vector3> p{cnoid::Vector3(0.12,0.05,0),cnoid::Vector3(0.12,-0.05,0),cnoid::Vector3(-0.09,0.05,0),cnoid::Vector3(-0.09,-0.05,0)};
      for(int i=0;i<p.size();i++){
        std::shared_ptr<aik_constraint::Force> force = std::make_shared<aik_constraint::Force>();
        force->A_link() = robot->link("RLEG_ANKLE_R");
        force->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.045) + p[i];
        force->B_link() = nullptr;
        aik_constraint::Force::setSPAN(force,0.2);
        forces.push_back(force);

        std::shared_ptr<aik_constraint::ForceConstraint> constraint = std::make_shared<aik_constraint::ForceConstraint>();
        constraint->force() = force;
        constraint->dl() = Eigen::VectorXd::Zero(4);
        constraint->C().resize(4,4);
        constraint->du() = 1e10 * Eigen::VectorXd::Ones(4);
        for(int j=0;j<4;j++) constraint->C().insert(j,j) = 1.0;
        constraints0.push_back(constraint);
      }
    }
    {
      // rleg
      std::vector<cnoid::Vector3> p{cnoid::Vector3(0.12,0.05,0),cnoid::Vector3(0.12,-0.05,0),cnoid::Vector3(-0.09,0.05,0),cnoid::Vector3(-0.09,-0.05,0)};
      for(int i=0;i<p.size();i++){
        std::shared_ptr<aik_constraint::Force> force = std::make_shared<aik_constraint::Force>();
        force->A_link() = robot->link("LLEG_ANKLE_R");
        force->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.045) + p[i];
        force->B_link() = nullptr;
        aik_constraint::Force::setSPAN(force,0.2);
        forces.push_back(force);

        std::shared_ptr<aik_constraint::ForceConstraint> constraint = std::make_shared<aik_constraint::ForceConstraint>();
        constraint->force() = force;
        constraint->dl() = Eigen::VectorXd::Zero(4);
        constraint->C().resize(4,4);
        constraint->du() = 1e10 * Eigen::VectorXd::Ones(4);
        for(int j=0;j<4;j++) constraint->C().insert(j,j) = 1.0;
        constraints0.push_back(constraint);
      }
    }

    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > constraints1;
    {
      // task: EOM
      std::shared_ptr<aik_constraint::EOMConstraint> constraint = std::make_shared<aik_constraint::EOMConstraint>();
      constraint->robot() = robot;
      constraints1.push_back(constraint);
    }

    int debugLevel = 2; // 0 or 1 or 2
    std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
    std::vector<cnoid::LinkPtr> variables;
    std::vector<std::vector<std::shared_ptr<aik_constraint::IKConstraint> > > constraints{constraints0,constraints1};
    for(size_t i=0;i<constraints.size();i++){
      for(size_t j=0;j<constraints[i].size();j++){
        constraints[i][j]->debugLevel() = debugLevel;//debug
      }
    }

    prioritized_acc_inverse_kinematics_solver::IKParam param;
    param.debugLevel = debugLevel;
    bool solved = prioritized_acc_inverse_kinematics_solver::solveAIK(variables,
                                                                      forces,
                                                                      constraints,
                                                                      tasks,
                                                                      param);
    if(!solved) std::cerr << "!solved" << std::endl;
    else std::cerr << "solved" << std::endl;

    // visualize
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

    return;
  }
}
