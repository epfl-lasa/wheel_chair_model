#include <Read_obstacle_state.h>

void gazebo::Read_obstacle_state::chatterCallback_desired_state_2D(const geometry_msgs::Pose2D & msg)
{
  // obstacle_pos_desired.pos.x=msg.x;
  // obstacle_pos_desired.pos.y=msg.y;
  obstacle_pos_desired.Set(msg.x, msg.y, 0, 0, 0, 0);
  position_has_not_recieved=false;
}

gazebo::Read_obstacle_state::Read_obstacle_state()
{

}

void gazebo::Read_obstacle_state::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	model = _model;
	// Initialize ros
	if (!ros::isInitialized())
	{
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "Read_joint_state",
				ros::init_options::NoSigintHandler);
	}

	rosNode = new ros::NodeHandle("Read_obstacle_Position");
	pubobstacle = rosNode->advertise<geometry_msgs::Pose2D>(model->GetName(), 1000);
	std::string str= "Desired_Pose_";
	std::string str2=model->GetName();
	str.append(str2);
	sub_desired_state_2D = rosNode->subscribe(str,3, &Read_obstacle_state::chatterCallback_desired_state_2D, this);

	// Safety check
	std::cerr << "\nThe Read_joint_position plug-in is attach to model["
			<< _model->GetName() << "]\n";
	position_has_not_recieved=true;

	// Commented because this does delay gazebo start up
	// while (position_has_not_recieved)
	// {
		// ros::spinOnce();
		// cout<<"Waiting for the position of the obstacle. Position of "<<model->GetName()<<" has not been received"<<endl;
		// ros::Duration(0.1).sleep(); // sleep for half a second -- does not work before revieving obstacle
	// }

	OnUpdateThread = std::thread(std::bind(&Read_obstacle_state::OnUpdate, this));
}

void gazebo::Read_obstacle_state::OnUpdate()
{
	ros::Rate r(100); // 100 hz

	while (rosNode->ok())
	{
          // obstacle_pos = model->GetWorldPose();
		// mObstacle_pos.x = obstacle_pos.pos.x;
		// mObstacle_pos.y = obstacle_pos.pos.y;
          obstacle_pos = model->WorldPose();
                mObstacle_pos.x = obstacle_pos.Pos().X();
                mObstacle_pos.y = obstacle_pos.Pos().Y();
                
		// mObstacle_pos.theta = obstacle_pos.rot.GetAsEuler().z;
                mObstacle_pos.theta = obstacle_pos.Rot().Euler()[2];
		pubobstacle.publish(mObstacle_pos);
		model->SetWorldPose(obstacle_pos_desired);
		r.sleep();
	}
}
