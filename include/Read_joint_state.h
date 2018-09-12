/*
 * read_joint_state.h
 *
 *  Created on: Sep 11, 2018
 *      Author: sina
 */

#ifndef WHEEL_CHAIR_MODEL_INCLUDE_READ_JOINT_STATE_H_
#define WHEEL_CHAIR_MODEL_INCLUDE_READ_JOINT_STATE_H_


#include <gazebo/gazebo.hh>
#include "eigen3/Eigen/Dense"
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "sensor_msgs/JointState.h"
using namespace std;
using namespace Eigen;

namespace gazebo
{
/// \brief A plugin to read joint positions sensor.
class Read_joint_state : public ModelPlugin
{

	/// \brief Constructor
public:
	Read_joint_state();

/// \brief The load function is called by Gazebo when the plugin is
/// inserted into simulation
/// \param[in] _model A pointer to the model that this plugin is
/// attached to.
/// \param[in] _sdf A pointer to the plugin's SDF element.
	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

// Called by the world update start event
	virtual void OnUpdate();

private:

	sensor_msgs::JointState mJoint_state;
	ros::NodeHandle *rosNode;
	ros::Publisher pubJoint_state;


	int number_of_joints;
	VectorXd Current_joint_position;
	VectorXd Current_joint_velocity;
	vector<string> Joint_names;

	physics::ModelPtr model; /// \brief Pointer to the model.

	physics::JointPtr joint; /// \brief Pointer to the joint.

	std::thread OnUpdateThread;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(Read_joint_state)
}




#endif /* WHEEL_CHAIR_MODEL_INCLUDE_READ_JOINT_STATE_H_ */
