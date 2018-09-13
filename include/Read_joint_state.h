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

	void 						chatterCallback_desired_state_2D(const sensor_msgs::JointState & msg);
	void 						chatterCallback_control_level(const std_msgs::Int64 & msg);
	void 						chatterCallback_desired_state_complete(const sensor_msgs::JointState & msg);
	void 						chatterCallback_desired_tele_state(const geometry_msgs::Twist & msg);


	std::thread 				OnUpdateThread;

	ENUM_Control_Type 			Control_Level;

	sensor_msgs::JointState 	mJoint_state;
	geometry_msgs::Pose2D		mWheelchair;
	ros::NodeHandle 			*rosNode;

	ros::Publisher 				pubJoint_state;
	ros::Publisher 				pubWheelchair_state;
	ros::Publisher 				pubObstacle_state;
	ros::Subscriber 			sub_desired_state_2D;
	ros::Subscriber 			sub_control_level;
	ros::Subscriber 			sub_desired_state_complete;
	ros::Subscriber 			sub_desired_tele_state;


	int							number_of_joints;
	vector<string> 				Joint_names;
	VectorXd 					Current_joint_position;
	VectorXd 					Current_joint_velocity;
	VectorXd 					Desired_Position_complete;
	VectorXd 					Desired_Velocity_complete;
	VectorXd 					Desired_torque_complete;
	VectorXd 					Desired_Tele_State;
	VectorXd 					Desired_Position_2D;
	VectorXd 					Desired_Velocity_2D;
	VectorXd 					Desired_torque_2D;
	math::Pose					Wheelchair_pos;

	physics::ModelPtr			model; /// \brief Pointer to the model.
	physics::JointPtr 			joint; /// \brief Pointer to the joint.

};

// Tell Gazebo about this plug-in, so that Gazebo can call Load on this plug-in.
GZ_REGISTER_MODEL_PLUGIN(Read_joint_state)
}




#endif /* WHEEL_CHAIR_MODEL_INCLUDE_READ_JOINT_STATE_H_ */
