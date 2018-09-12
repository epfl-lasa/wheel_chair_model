
#include <Read_joint_state.h>


gazebo::Read_joint_state::Read_joint_state()
{

}
void gazebo::Read_joint_state::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	// Initialize ros
	if (!ros::isInitialized())
	{
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "Read_joint_state",
				ros::init_options::NoSigintHandler);
	}
	rosNode =new ros::NodeHandle("Read_joint_state");
	pubJoint_state= rosNode->advertise<sensor_msgs::JointState>("quickie_wheel_states", 1000);

	// Safety check
	std::cerr << "\nThe Read_joint_position plugin is attach to model[" <<
			_model->GetName() << "]\n";
	if (_model->GetJointCount() == 0)
	{
		std::cerr << "Invalid joint count, the read joint plugin is not loaded\n";
		return;
	}

	// Store the model pointer for convenience.
	model = _model;
	number_of_joints=0;

	// Get the joint names and their initial positions.
	number_of_joints =model->GetJointCount();

	cout<<"Number of Joints are "<<number_of_joints<<endl;
	Current_joint_position.resize(number_of_joints);
	Current_joint_velocity.resize(number_of_joints);
	for(int i=0;i<number_of_joints;i++)
	{
		joint = model->GetJoints()[i];
		Joint_names.push_back(joint->GetName());
		mJoint_state.name.push_back(joint->GetName());
		Current_joint_position(i)=joint->GetAngle(0).Radian();
		Current_joint_velocity(i)=joint->GetVelocity(0);
		cout<<"Name of joint "<<i<<" is: "<<joint->GetName()<<endl;
	}
	cout<<"Joint positions"<<Current_joint_position<<endl;
	cout<<"Joint velocity"<<Current_joint_velocity<<endl;
	mJoint_state.position.resize(number_of_joints);
	mJoint_state.velocity.resize(number_of_joints);
	mJoint_state.name.resize(number_of_joints);
	OnUpdateThread =std::thread(std::bind(&Read_joint_state::OnUpdate, this));
}


void gazebo::Read_joint_state::OnUpdate()
{
	ros::Rate r(100); // 100 hz
	while (rosNode->ok())
	{
		for(int i=0;i<number_of_joints;i++)
		{
			Current_joint_position(i)=model->GetJoints()[i]->GetAngle(0).Radian();
			Current_joint_velocity(i)=model->GetJoints()[i]->GetVelocity(0);
			mJoint_state.position[i]=Current_joint_position(i);
			mJoint_state.velocity[i]=Current_joint_velocity(i);
		}

		pubJoint_state.publish(mJoint_state);
		r.sleep();
	}


}
