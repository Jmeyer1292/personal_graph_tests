#include <ros/ros.h>

#include <descartes_planner/graph_builder.h>
#include "descartes_planner/dense_planner.h"
#include "descartes_trajectory/axial_symmetric_pt.h"
#include <descartes_utilities/ros_conversions.h>

#include "ur5_demo_descartes/ur5_robot_model.h"

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

static bool createLine(const Eigen::Vector3d &start, const Eigen::Vector3d &stop, int num_points, EigenSTL::vector_Affine3d &poses)
{
  Eigen::Vector3d delta = (stop - start) / num_points;

  Eigen::AngleAxisd flip_z (M_PI, Eigen::Vector3d::UnitY());

  for (int i = 0; i < num_points; ++i)
  {
    Eigen::Affine3d pose = Eigen::Affine3d::Identity() * flip_z;
    pose.translation() = (start + i * delta);
    poses.push_back(pose);
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "graph_tests");
  ros::AsyncSpinner spinner (1);
  spinner.start();

  auto kin_model = boost::make_shared<ur5_demo_descartes::UR5RobotModel>();
  assert(kin_model->initialize("robot_description", "manipulator", "world", "tool"));

  using DescartesTraj = std::vector<descartes_core::TrajectoryPtPtr>;

  DescartesTraj output;



  Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
  orientation = orientation * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());

  descartes_planner::ConstrainedSegment segment;
  segment.start = Eigen::Vector3d(0.3, 0.3, 0);
  segment.end = Eigen::Vector3d(0.3, 0.1, 0);
  segment.linear_disc = 0.02;
  segment.linear_vel = 0.10;
  segment.z_axis_disc = 0.3;
//  segment.orientations.push_back(orientation);

  orientation = orientation * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX());
  segment.orientations.push_back(orientation);



  auto graph = descartes_planner::sampleConstrainedPaths(*kin_model, segment);

  descartes_planner::PlanningGraph plan_graph (kin_model);
  plan_graph.setGraph(graph);

  std::list<descartes_trajectory::JointTrajectoryPt> sol;
  double cost=  -1.0;
  plan_graph.getShortestPath(cost, sol);

  ROS_WARN_STREAM("SEARCH COMPLETE: Cost = " << cost << " and length = " << sol.size());

  std::for_each(sol.begin(), sol.end(), [&output] (const descartes_trajectory::JointTrajectoryPt& pt)
  {
    auto it = boost::make_shared<descartes_trajectory::JointTrajectoryPt>(pt);
    output.push_back(it);
  });

  trajectory_msgs::JointTrajectory traj;
  descartes_utilities::toRosJointPoints(*kin_model, output, 1.0, traj.points);
  traj.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = "";

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/joint_trajectory_action", true);

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = traj;

  if (!ac.waitForServer(ros::Duration(3.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return 1;
  }

  ROS_INFO("Sending initial trajectory");
  ac.sendGoal(goal);
  ac.waitForResult();

  ros::waitForShutdown();
}
