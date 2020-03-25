/*
 * read_joint_state.h
 *
 *  Created on: Sep 11, 2018
 *      Author: sina
 */

#ifndef WHEEL_CHAIR_MODEL_INCLUDE_READ_OBSTACLE_STATE_H_
#define WHEEL_CHAIR_MODEL_INCLUDE_READ_OBSTACLE_STATE_H_

#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
// #include <gazebo/math/gzmath.hh>
// #include <gazebo/Math.hh>
#include "eigen3/Eigen/Dense"
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"

using namespace std;
using namespace Eigen;

enum ENUM_Control_Type{CONTROL_NONE=0, CONTROL_POS_2D, CONTROL_VEL_2D, CONTROL_TORQUE_2D, CONTROL_POS_COM, CONTROL_VEL_COM, CONTROL_TORQUE_COM, CONTROL_TELE};


namespace gazebo
{
/// \brief A plugin to read joint positions sensor.
class Read_obstacle_state : public ModelPlugin
{

	/// \brief Constructor
public:
	Read_obstacle_state();

/// \brief The load function is called by Gazebo when the plugin is
/// inserted into simulation
/// \param[in] _model A pointer to the model that this plugin is
/// attached to.
/// \param[in] _sdf A pointer to the plugin's SDF element.
	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

// Called by the world update start event
	virtual void OnUpdate();

private:

	void 						chatterCallback_desired_state_2D(const geometry_msgs::Pose2D & msg);
	std::thread 				OnUpdateThread;
	geometry_msgs::Pose2D		mObstacle_pos;

	ros::NodeHandle 			*rosNode;
	ros::Publisher 				pubobstacle;
	ros::Subscriber				sub_desired_state_2D;

  // gazebo::math::Pose					obstacle_pos;
  // gazebo::math::Pose					obstacle_pos_desired;
  ignition::math::Pose3<double>				obstacle_pos;
  ignition::math::Pose3<double>				obstacle_pos_desired;
	bool 						position_has_not_recieved;

	physics::ModelPtr			model; /// \brief Pointer to the model.

};

// Tell Gazebo about this plug-in, so that Gazebo can call Load on this plug-in.
GZ_REGISTER_MODEL_PLUGIN(Read_obstacle_state)
}




#endif /* WHEEL_CHAIR_MODEL_INCLUDE_READ_JOINT_STATE_H_ */
