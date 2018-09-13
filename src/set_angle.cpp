#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#include <gazebo/gazebo.hh>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include <memory>
#include <map>

namespace gazebo
{
    class force_joint : public ModelPlugin
    {
    public:
        force_joint() : ModelPlugin(){}

        //read every joint of model and initialize ROS topics
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            this->model = _parent;
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&force_joint::OnUpdate, this, _1));
            physics::Joint_V jointVector = this->model->GetJoints();

            // Initialize ros, if it has not already bee initialized.
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client",
                ros::init_options::NoSigintHandler);
            }

            // Create our ROS node. This acts in a similar manner to the Gazebo node
            this->rosNode.reset(new ros::NodeHandle("force_joints"));

            // Create a named topic, and subscribe to it for every joint
            for(physics::Joint_V::iterator jit=jointVector.begin(); jit!=jointVector.end(); ++jit)
            {
                //test if revolute joint ... i think ...
                //if not, ignore joint
                if((*jit)->GetType() != 576)
                    continue;
                std::string modelName = this->model->GetName();
                boost::algorithm::replace_all(modelName, " ", "_");
                std::string jointName = (*jit)->GetName();
                boost::algorithm::replace_all(jointName, " ", "_");
                std::string subPath = modelName + "/" + jointName;
                ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(subPath,
                                                                            1,
                                                                            boost::bind(&force_joint::OnRosMsg, this, _1, (*jit)->GetName()),
                                                                            ros::VoidPtr(), &this->rosQueue);
                this->rosSubList.push_back(this->rosNode->subscribe(so));
                joints.push_back((*jit)->GetName());
                jointAngles[(*jit)->GetName()] = (*jit)->GetAngle(0).Radian();
            }

            // Spin up the queue helper thread
            this->rosQueueThread = std::thread(std::bind(&force_joint::QueueThread, this));
        }

        //Handle an incoming message from ROS
        void OnRosMsg(const std_msgs::Float32ConstPtr &_msg, std::string jointName)
        {
            this->jointAngles[jointName] = _msg->data;
            return;
        }

        void OnUpdate(const common::UpdateInfo & _info)

        {
            //set velocity and force to zero for every saved joint
            //and set angle to saved value
            for(auto it=this->joints.begin(); it!=joints.end(); ++it)
            {
                this->model->GetJoint(*it)->SetVelocity(0, 0);
                this->model->GetJoint(*it)->SetForce(0, 0);
                this->model->GetJoint(*it)->SetAngle(0, math::Angle(jointAngles[*it]));
            }
            return;
        }
    private:
        //ROS helper function that processes messages
        void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        physics::ModelPtr model;

        event::ConnectionPtr updateConnection;

        std::unique_ptr<ros::NodeHandle> rosNode;

        //List with all subscribers
        std::list<ros::Subscriber> rosSubList;

        //List with all revolute joint names
        std::list<std::string> joints;

        ros::CallbackQueue rosQueue;

        std::thread rosQueueThread;

        //map with joints' angles by their names
        std::map<std::string, double> jointAngles;
    };

    GZ_REGISTER_MODEL_PLUGIN(force_joint)
}
