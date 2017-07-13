#include <ros/ros.h>

#include <descartes_planner/graph_builder.h>
#include "descartes_planner/dense_planner.h"
#include "descartes_trajectory/axial_symmetric_pt.h"
#include <descartes_utilities/ros_conversions.h>
#include "ur5_demo_descartes/ur5_robot_model.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "graph_tests");
  ros::AsyncSpinner spinner (1);
  spinner.start();

  auto kin_model = boost::make_shared<ur5_demo_descartes::UR5RobotModel>();
  kin_model->initialize("robot_description", "manipulator", "world", "tool");

  using DescartesTraj = std::vector<descartes_core::TrajectoryPtPtr>;

  DescartesTraj output;

  // So the idea is that we create some sequence of linear paths with a defined set of
  // valid orientations. Each of these is a 'ConstrainedSegment'.
  descartes_planner::ConstrainedSegment segment;
  segment.start = Eigen::Vector3d(0.3, 0.3, 0); // Start position (m)
  segment.end = Eigen::Vector3d(0.3, 0.1, 0); // End position (m)
  segment.linear_disc = 0.02; // approximate linear discretization (m)
  segment.linear_vel = 0.10; // approximate linear velocity (m/s)
  segment.z_axis_disc = 0.1; // angle discretization about Z (radians)

  // compute nominal orientation for that path
  Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
  orientation = orientation * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
  segment.orientations.push_back(orientation); // a vector of 3x3 rotation matrices

  auto segment2 = segment; // create a second segment
  segment2.start = segment.end;
  segment2.end = Eigen::Vector3d(0.3, 0.4, 0.3);

  auto segment3 = segment2; // create a third segment
  segment3.start= segment2.end;
  segment3.end = Eigen::Vector3d(0.3, 0.1, 0.0);
  orientation = orientation * Eigen::AngleAxisd(0.3, Eigen::Vector3d(1, 0, 0)); // tweak the orientation
  segment3.orientations[0] = orientation;

  // Now for each segment, we build a 'graph' that describes its solutions
  auto graph = descartes_planner::sampleConstrainedPaths(*kin_model, segment);
  auto graph2 = descartes_planner::sampleConstrainedPaths(*kin_model, segment2);
  auto graph3 = descartes_planner::sampleConstrainedPaths(*kin_model, segment3);

  // Then we stack the graphs end to end
  // !!!NOTE!!!: You have to connect the last rung of the first graph with the first rung of the next graph
  // I do this naively in this function by connecting ALL starting positions and weighting them by L1 joint
  // distance. You could perhaps write your own function that generates connections based on some other logic.
  descartes_planner::appendInTime(graph, graph2);
  descartes_planner::appendInTime(graph, graph3);

  // Create a planning graph (it has a solve method - you could use the DagSearch class yourself if you wanted)
  descartes_planner::PlanningGraph plan_graph (kin_model);
  plan_graph.setGraph(graph); // set the graph we built earlier (instead of calling insertGraph)

  // Retrieve the solution
  std::list<descartes_trajectory::JointTrajectoryPt> sol;
  double cost=  -1.0;
  plan_graph.getShortestPath(cost, sol);

  ROS_WARN_STREAM("SEARCH COMPLETE: Cost = " << cost << " and length = " << sol.size());

  // For legacy reasons this function returns a list ( -_- )... so we put it in a vector
  std::for_each(sol.begin(), sol.end(), [&output] (const descartes_trajectory::JointTrajectoryPt& pt)
  {
    // and we convert it to a shared pointer
    auto it = boost::make_shared<descartes_trajectory::JointTrajectoryPt>(pt);
    output.push_back(it);
  });


  // Everything here on down is just boiler plate crap for publishing the trajectory
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
