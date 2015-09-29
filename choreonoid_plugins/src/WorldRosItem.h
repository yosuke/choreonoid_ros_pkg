#ifndef CNOID_ROS_PLUGIN_WORLD_ROS_ITEM_H_INCLUDED
#define CNOID_ROS_PLUGIN_WORLD_ROS_ITEM_H_INCLUDED

#include <cnoid/Item>
#include <cnoid/Body>
#include <cnoid/WorldItem>
#include <cnoid/SimulatorItem>
#include <cnoid/TimeBar>
#include "exportdecl.h"

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Wrench.h>
#include <std_srvs/Empty.h>
#include <rosgraph_msgs/Clock.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>

#include <vector>
#include <boost/thread.hpp>

namespace cnoid {

class CNOID_EXPORT WorldRosItem : public Item
{
public:
    static void initialize(ExtensionManager* ext);
    
    WorldRosItem();
    WorldRosItem(const WorldRosItem& org);
    virtual ~WorldRosItem();
    
    void start();
    void stop();
    
    void setModuleName(const std::string& name);

protected:
    virtual ItemPtr doDuplicate() const;
    
private:
    WorldItemPtr world;
    SimulatorItemPtr sim;
    boost::shared_ptr<ros::NodeHandle> rosnode_;
    boost::shared_ptr<ros::AsyncSpinner> async_ros_spin_;
    ros::CallbackQueue rosqueue_;
    boost::shared_ptr<boost::thread> rosqueue_thread_;

    bool timeTick(double);
    void publishSimTime();
    void publishLinkStates();
    void publishModelStates();
    void queueThread();
    bool resetSimulation(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool pausePhysics(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool unpausePhysics(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool spawnVRMLModel(gazebo_msgs::SpawnModel::Request &req, gazebo_msgs::SpawnModel::Response &res);
    bool deleteModel(gazebo_msgs::DeleteModel::Request &req, gazebo_msgs::DeleteModel::Response &res);

    ros::Publisher     pub_clock_;
    ros::Publisher     pub_link_states_;
    ros::Publisher     pub_model_states_;
    ros::ServiceServer reset_simulation_service_;
    ros::ServiceServer reset_world_service_;
    ros::ServiceServer pause_physics_service_;
    ros::ServiceServer unpause_physics_service_;
    ros::ServiceServer spawn_vrml_model_service_;
    ros::ServiceServer delete_model_service_;
};

typedef ref_ptr<WorldRosItem> WorldRosItemPtr;
}

#endif
