/*
 * read_joint_state.h
 *
 *  Created on: Sep 11, 2018
 *      Author: sina
 */

#ifndef WHEEL_CHAIR_MODEL_INCLUDE_READ_TORQUE_FORCE_H_
#define WHEEL_CHAIR_MODEL_INCLUDE_READ_TORQUE_FORCE_H_


#include <gazebo/gazebo.hh>
#include "eigen3/Eigen/Dense"
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/sensors.hh>
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
class Read_torque_force : public SensorPlugin
{

	/// \brief Constructor
public:
	Read_torque_force();

/// \brief The load function is called by Gazebo when the plugin is
/// inserted into simulation
/// \param[in] _model A pointer to the model that this plugin is
/// attached to.
/// \param[in] _sdf A pointer to the plugin's SDF element.
	virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

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

	sensors::ForceTorqueSensorPtr parentSensor;
	event::ConnectionPtr 		  updateConnection;

	std::thread OnUpdateThread;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_SENSOR_PLUGIN(Read_torque_force)
}




#endif /* WHEEL_CHAIR_MODEL_INCLUDE_READ_JOINT_STATE_H_ */
