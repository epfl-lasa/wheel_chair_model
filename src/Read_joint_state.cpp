#include <Read_joint_state.h>

void gazebo::Read_joint_state::chatterCallback_desired_state_2D(
		const sensor_msgs::JointState & msg)
{
	switch (Control_Level)
	{
	case 1:
		Desired_Position_2D(0) = msg.position[0];
		Desired_Position_2D(1) = msg.position[1];
		break;
	case 2:
		Desired_Velocity_2D(0) = msg.velocity[0];
		Desired_Velocity_2D(1) = msg.velocity[1];
		break;
	case 3:
		Desired_torque_2D(0) = msg.effort[0];
		Desired_torque_2D(1) = msg.effort[1];
		break;
	}

}
void gazebo::Read_joint_state::chatterCallback_desired_state_complete(
		const sensor_msgs::JointState & msg)
{
	switch (Control_Level)
	{
	case 4:
		for (int i = 0; i < number_of_joints; i++)
		{
			Desired_Position_complete(i) = msg.position[i];
		}
		break;
	case 5:
		for (int i = 0; i < number_of_joints; i++)
		{
			Desired_Velocity_complete(i) = msg.velocity[i];
		}
		break;
	case 6:
		for (int i = 0; i < number_of_joints; i++)
		{
			Desired_torque_complete(i) = msg.effort[i];
		}
		break;
	}

}

void gazebo::Read_joint_state::chatterCallback_control_level(
		const std_msgs::Int64 & msg)
{
	if (Control_Level == 0)
	{
		switch (msg.data)
		{
		case 0:
			Control_Level = CONTROL_NONE;
			cout << "Control level is: " << " NONE" << endl;
			break;
		case 1:
			Control_Level = CONTROL_POS_2D;
			cout << "Control level is: " << " CONTROL_POS_2D" << endl;
			break;
		case 2:
			Control_Level = CONTROL_VEL_2D;
			cout << "Control level is: " << " CONTROL_VEL_2D" << endl;
			break;
		case 3:
			Control_Level = CONTROL_TORQUE_2D;
			cout << "Control level is: " << "CONTROL_TORQUE_2D" << endl;
			break;
		case 4:
			Control_Level = CONTROL_POS_COM;
			cout << "Control level is: " << " CONTROL_POS_COM" << endl;
			break;
		case 5:
			Control_Level = CONTROL_VEL_COM;
			cout << "Control level is: " << " CONTROL_VEL_COM" << endl;
			break;
		case 6:
			Control_Level = CONTROL_TORQUE_COM;
			cout << "Control level is: " << " CONTROL_TORQUE_COM" << endl;
			break;
		}
	}

}

void gazebo::Read_joint_state::chatterCallback_desired_tele_state(
		const geometry_msgs::Twist & msg)
{
	if (Control_Level == CONTROL_TELE)
	{
		Desired_Tele_State(0) = msg.linear.x;
		Desired_Tele_State(1) = msg.angular.z;
	}
}

gazebo::Read_joint_state::Read_joint_state()
{

}
void gazebo::Read_joint_state::Load(physics::ModelPtr _model,
		sdf::ElementPtr _sdf)
{
	// Initialize ros
	if (!ros::isInitialized())
	{
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "Read_joint_state",
				ros::init_options::NoSigintHandler);
	}
	rosNode = new ros::NodeHandle("Read_joint_state");
	pubJoint_state = rosNode->advertise<sensor_msgs::JointState>(
			"quickie_wheel_states", 1000);
	pubWheelchair_state = rosNode->advertise<geometry_msgs::Pose2D>("quickie_state", 1000);
	sub_desired_state_2D = rosNode->subscribe("/quickie_wheel_Desired_state_2D",
			3, &Read_joint_state::chatterCallback_desired_state_2D, this);
	sub_desired_state_complete = rosNode->subscribe(
			"/quickie_wheel_Desired_state_complete", 3,
			&Read_joint_state::chatterCallback_desired_state_complete, this);
	sub_desired_tele_state = rosNode->subscribe("/cmd_vel", 3,
			&Read_joint_state::chatterCallback_desired_tele_state, this);
	sub_control_level = rosNode->subscribe("/quickie_wheel_Control_level", 3,
			&Read_joint_state::chatterCallback_control_level, this);

	// Safety check
	std::cerr << "\nThe Read_joint_position plugin is attach to model["
			<< _model->GetName() << "]\n";
	if (_model->GetJointCount() == 0)
	{
		std::cerr
				<< "Invalid joint count, the read joint plugin is not loaded\n";
		return;
	}

	// Store the model pointer for convenience.
	model = _model;
	number_of_joints = 0;

	// Get the joint names and their initial positions.
	number_of_joints = model->GetJointCount();

	cout << "Number of Joints are " << number_of_joints << endl;
	Current_joint_position.resize(number_of_joints);
	Current_joint_velocity.resize(number_of_joints);
	Desired_Position_complete.resize(number_of_joints);
	Desired_Velocity_complete.resize(number_of_joints);
	Desired_torque_complete.resize(number_of_joints);
	Desired_Position_2D.resize(2);
	Desired_Velocity_2D.resize(2);
	Desired_torque_2D.resize(2);
	Desired_Tele_State.resize(2);
	mWheelchair.theta=0;
	mWheelchair.x=0;
	mWheelchair.y=0;

	for (int i = 0; i < number_of_joints; i++)
	{
		joint = model->GetJoints()[i];
		Joint_names.push_back(joint->GetName());
		mJoint_state.name.push_back(joint->GetName());
		Current_joint_position(i) = joint->GetAngle(0).Radian();
		Current_joint_velocity(i) = joint->GetVelocity(0);
		cout << "Name of joint " << i << " is: " << joint->GetName() << endl;
	}
	cout << "Joint positions" << Current_joint_position << endl;
	cout << "Joint velocity" << Current_joint_velocity << endl;
	mJoint_state.position.resize(number_of_joints);
	mJoint_state.velocity.resize(number_of_joints);
	mJoint_state.name.resize(number_of_joints);

	//Control_Level = CONTROL_NONE;
	//Control_Level = CONTROL_TORQUE_2D;
	Control_Level = CONTROL_TELE;
	ros::Rate r(5); // 100 hz
	int counter;
	while ((Control_Level == CONTROL_NONE) && (counter < 50))
	{
		ros::spinOnce();
		cout
				<< "Waiting for the control level to be set. CONTROL_POS_2D=1, CONTROL_VEL_2D=2, CONTROL_TORQUE_2D=3, CONTROL_POS_COM=4, CONTROL_VEL_COM=5, CONTROL_TORQUE_COM=6 "
				<< endl;
		r.sleep();
		counter = counter + 1;
	}

	OnUpdateThread = std::thread(std::bind(&Read_joint_state::OnUpdate, this));
}

void gazebo::Read_joint_state::OnUpdate()
{
	ros::Rate r(100); // 100 hz

	while (rosNode->ok())
	{
		for (int i = 0; i < number_of_joints; i++)
		{
			Current_joint_position(i) =
					model->GetJoints()[i]->GetAngle(0).Radian();
			Current_joint_velocity(i) = model->GetJoints()[i]->GetVelocity(0);
			mJoint_state.position[i] = Current_joint_position(i);
			mJoint_state.velocity[i] = Current_joint_velocity(i);
		}
		pubJoint_state.publish(mJoint_state);
		Wheelchair_pos=model->GetWorldPose();
		mWheelchair.x=Wheelchair_pos.pos.x;
		mWheelchair.y=Wheelchair_pos.pos.y;
		mWheelchair.theta=Wheelchair_pos.rot.GetAsEuler().z;
		pubWheelchair_state.publish(mWheelchair);
		switch (Control_Level)
		{
		case CONTROL_POS_2D:
			model->GetJoints()[0]->SetPosition(0, Desired_Position_2D(0));
			model->GetJoints()[1]->SetPosition(0, -Desired_Position_2D(0));
			model->GetJoints()[2]->SetPosition(0, -Desired_Position_2D(1));
			//model->GetJoints()[3]->SetPosition(0, 0);
			model->GetJoints()[4]->SetPosition(0, -Desired_Position_2D(1));
			//model->GetJoints()[5]->SetPosition(0, 0);
			//model->GetJoints()[6]->SetPosition(0, 0);
			//model->GetJoints()[7]->SetPosition(0, 0);
			model->GetJoints()[8]->SetPosition(0, Desired_Position_2D(1));
			model->GetJoints()[9]->SetPosition(0, Desired_Position_2D(1));
			break;
		case CONTROL_VEL_2D:
			break;
		case CONTROL_TORQUE_2D:
			model->GetJoints()[0]->SetForce(0, Desired_torque_2D(0)); // The big wheels should have the same torque
			model->GetJoints()[1]->SetForce(0, -Desired_torque_2D(0)); // The big wheels should have the same torque
			model->GetJoints()[2]->SetForce(0, -Desired_torque_2D(1)); // The pin of the front and the rear wheel can accept a torque
			model->GetJoints()[3]->SetForce(0, 0); // The front and the rear wheel do not accept a torque, they are just free wheels
			model->GetJoints()[4]->SetForce(0, -Desired_torque_2D(1)); // The pin of the front and the rear wheel can accept a torque
			model->GetJoints()[5]->SetForce(0, 0); // The front and the rear wheel do not accept a torque, they are just free wheels
			model->GetJoints()[6]->SetForce(0, 0); // The front and the rear wheel do not accept a torque, they are just free wheels
			model->GetJoints()[7]->SetForce(0, 0); // The front and the rear wheel do not accept a torque, they are just free wheels
			model->GetJoints()[8]->SetForce(0, Desired_torque_2D(1)); // The pin of the front and the rear wheel can accept a torque
			model->GetJoints()[9]->SetForce(0, Desired_torque_2D(1)); // The pin of the front and the rear wheel can accept a torque
			break;
		case CONTROL_POS_COM:
			break;
		case CONTROL_VEL_COM:
			break;
		case CONTROL_TORQUE_COM:
			for (int i = 0; i < number_of_joints; i++)
			{
				model->GetJoints()[i]->SetForce(0, Desired_torque_complete(i));
			}
			break;
		case CONTROL_TELE:
			model->GetJoints()[0]->SetVelocity(0,Desired_Tele_State(0)+Desired_Tele_State(1));
			model->GetJoints()[1]->SetVelocity(0,-Desired_Tele_State(0)+Desired_Tele_State(1));
			model->GetJoints()[2]->SetPosition(0, 0);
//			model->GetJoints()[3]->SetVelocity(0, Desired_Tele_State(0));
			model->GetJoints()[4]->SetPosition(0, 0);
	//		model->GetJoints()[5]->SetVelocity(0, -Desired_Tele_State(0));
		//	model->GetJoints()[6]->SetVelocity(0, Desired_Tele_State(0));
		//	model->GetJoints()[7]->SetVelocity(0, -Desired_Tele_State(0));
			model->GetJoints()[8]->SetPosition(0, 0);
			model->GetJoints()[9]->SetPosition(0, 0);
/*			model->GetJoints()[2]->SetPosition(0, -Desired_Tele_State(1));
			model->GetJoints()[3]->SetVelocity(0, Desired_Tele_State(0));
			model->GetJoints()[4]->SetPosition(0, -Desired_Tele_State(1));
			model->GetJoints()[5]->SetVelocity(0, -Desired_Tele_State(0));
			model->GetJoints()[6]->SetVelocity(0, Desired_Tele_State(0));
			model->GetJoints()[7]->SetVelocity(0, -Desired_Tele_State(0));
			model->GetJoints()[8]->SetPosition(0, Desired_Tele_State(1));
			model->GetJoints()[9]->SetPosition(0, Desired_Tele_State(1));*/
			break;
		}
		r.sleep();
	}

}
