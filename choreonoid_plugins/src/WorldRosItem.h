/**
   @file WorldRosItem.h
   @author
 */

#ifndef CNOID_ROS_PLUGIN_WORLD_ROS_ITEM_H_INCLUDED
#define CNOID_ROS_PLUGIN_WORLD_ROS_ITEM_H_INCLUDED

#include <cnoid/Item>
#include <cnoid/Body>
#include <cnoid/DyBody>
#include <cnoid/WorldItem>
#include <cnoid/SimulatorItem>
#include <cnoid/AISTSimulatorItem>
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
#include <gazebo_msgs/ContactsState.h>

#include <vector>
#include <boost/thread.hpp>

namespace cnoid {

/**
   @brief This class is for accessing protected 'getCollisions' method in the SimulatorItem class.
   The use of this class is limited to this only. How to use, please see following example.

   @code
   SimulatorItemPtr               p;
   WorldRosSimulatorItemAccessor* sim_access;

   p          = <set SimulatorItem's instance>;
   sim_access = static_cast<WorldRosSimulatorItemAccessor*>(p.get());

   CollisionsLinkPairListPtr link_pairs = sim_access->get_collisions();

   if (link_pairs) {
     // This physics engine are collision output supported.
   } else {
     // This physics engine are not collision output supported.
   }
   @endcode

   @attention This class does not consider usage other than the contents described in the explanation.
 */
class CNOID_EXPORT WorldRosSimulatorItemAccessor : public SimulatorItem
{
public:
  WorldRosSimulatorItemAccessor() { }
  CollisionLinkPairListPtr get_collisions() { return getCollisions(); }
  virtual SimulationBody* createSimulationBody(Body* orgBody) { return 0; }
  virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies) { return true; }
  virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies) { return true; }
};

typedef ref_ptr<WorldRosSimulatorItemAccessor> WorldRosSimulatorItemAccessorPtr;

/**
   @brief
 */
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
    virtual Item* doDuplicate() const;
    
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
    bool spawnModel(gazebo_msgs::SpawnModel::Request &req, gazebo_msgs::SpawnModel::Response &res);
    bool deleteModel(gazebo_msgs::DeleteModel::Request &req, gazebo_msgs::DeleteModel::Response &res);

    ros::Publisher     pub_clock_;
    ros::Publisher     pub_link_states_;
    ros::Publisher     pub_model_states_;
    ros::ServiceServer reset_simulation_service_;
    ros::ServiceServer reset_world_service_;
    ros::ServiceServer pause_physics_service_;
    ros::ServiceServer unpause_physics_service_;
    ros::ServiceServer spawn_vrml_model_service_;
    ros::ServiceServer spawn_urdf_model_service_;
    ros::ServiceServer spawn_sdf_model_service_;
    ros::ServiceServer delete_model_service_;

    /*
      For Publish collision data. (contacts state)
     */

    /// Publisher of world contacts state. (link collisions pair)
    ros::Publisher pub_world_contacts_state_;
    /// The registration id of calling function from physics engine. (physics engine is SimulatorItem's subclass)
    int sim_func_regid;
    /// For getting collision data.
    WorldRosSimulatorItemAccessor* sim_access_;

    /**
       @brief Publish link conatcts state.
       This information is the calculation result in the physics engine. (e.g. AISTSimulatorItem etc)
     */
    void publishContactsState();
};

typedef ref_ptr<WorldRosItem> WorldRosItemPtr;

}
#endif /* #ifndef CNOID_ROS_PLUGIN_WORLD_ROS_ITEM_H_INCLUDED */
