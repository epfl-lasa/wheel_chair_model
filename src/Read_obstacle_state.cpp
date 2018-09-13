#include <Read_obstacle_state.h>



gazebo::Read_obstacle_state::Read_obstacle_state()
{

}
void gazebo::Read_obstacle_state::Load(physics::ModelPtr _model,
		sdf::ElementPtr _sdf)
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

	// Safety check
	std::cerr << "\nThe Read_joint_position plugin is attach to model["
			<< _model->GetName() << "]\n";

	OnUpdateThread = std::thread(std::bind(&Read_obstacle_state::OnUpdate, this));
}

void gazebo::Read_obstacle_state::OnUpdate()
{
	ros::Rate r(10); // 100 hz

	while (rosNode->ok())
	{
		obstacle_pos = model->GetWorldPose();
		mObstacle_pos.x = obstacle_pos.pos.x;
		mObstacle_pos.y = obstacle_pos.pos.y;
		mObstacle_pos.theta = obstacle_pos.rot.GetAsEuler().z;
		pubobstacle.publish(mObstacle_pos);
		r.sleep();
	}

}
