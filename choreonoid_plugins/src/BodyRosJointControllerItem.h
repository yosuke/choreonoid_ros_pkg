/**
  @file
  @author
 */

#ifndef CNOID_ROS_PLUGIN_BODY_ROS_JOINT_CONTROLLER_ITEM_H_INCLUDED
#define CNOID_ROS_PLUGIN_BODY_ROS_JOINT_CONTROLLER_ITEM_H_INCLUDED

#include <cnoid/ControllerItem>
#include <cnoid/Body>
#include <cnoid/BodyItem>
#include <cnoid/Link>
#include <cnoid/Archive>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include "exportdecl.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Wrench.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <vector>

#define DEBUG_ROS_JOINT_CONTROLLER 0

namespace cnoid {

class CNOID_EXPORT BodyRosJointControllerItem : public ControllerItem
{
public:
    /**
      @brief Constructor.
     */
    BodyRosJointControllerItem();

    /**
      @brief Copy constructor.
     */
    BodyRosJointControllerItem(const BodyRosJointControllerItem& org);

    /**
      @brief Destructor.
     */
    virtual ~BodyRosJointControllerItem();

    virtual bool start(Target* target);

    virtual double timeStep() const {
      return timeStep_;
    };

    virtual void input();
    virtual bool control();
    virtual void output();
    virtual void stop();

    /**
      @brief Receive joint trajectory message.
      this method is callback method of joint state subscribe.
      details joint trajectory message, see http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html
      @param[in] msg Joint trajectory message.
     */
    void receive_message(const trajectory_msgs::JointTrajectory::ConstPtr& msg);

    const BodyPtr& body() const {
      return simulationBody;
    };

    double controlTime() const {
      return controlTime_;
    }

    void setModuleName(const std::string& name);

protected:
    virtual Item* doDuplicate() const;
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
    void doPutProperties(PutPropertyFunction& putProperty);

    BodyPtr simulationBody;
    double timeStep_;

    const Target* controllerTarget;
    double controlTime_;
    std::ostream& os;

    std::string bodyName;

    boost::shared_ptr<ros::NodeHandle> rosnode_;
    boost::shared_ptr<ros::AsyncSpinner> async_ros_spin_;

    /**
      execute publish and subscribe, by topic of /[model_name]/[control_mode_name_]/[topic_name].
    */
    std::string control_mode_name_;

    sensor_msgs::JointState joint_state_;
    ros::Publisher joint_state_publisher_;
    ros::Subscriber joint_state_subscriber_;
    double joint_state_update_rate_;
    double joint_state_update_period_;
    double joint_state_last_update_;

    std::map<std::string, int> joint_number_map_;
    std::vector<std::string> joint_names_;
    std::vector<trajectory_msgs::JointTrajectoryPoint> points_;
    double trajectory_timestamp_;
    double trajectory_start_;
    unsigned int trajectory_index_;
    bool has_trajectory_;

    /*
      @brief Hook of simulation start.
      this method calling in BodyRosJointControllerItem::start.
      @retval true Start controller.
      @retval false Stop controller.
     */
    virtual bool hook_of_start();

    /**
      @brief Apply joint trajectory of inner buffer.
      this method calling in BodyRosJointControllerItem::control.
      @param[in] joint Pointer of joint. (cnoid::Link*)
      @param[in] idx Index of joint.
      this is trajectory message's joint index, not cnoid::Link::jointId().
      @param[in] point Pointer of joint trajectory point message.
      details see http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html
     */
    virtual void apply_message(Link* joint, size_t idx, trajectory_msgs::JointTrajectoryPoint* point);

    /**
      @brief Keep the attitude.
      this method calling in BodyRosJointControllerItem::control.
     */
    virtual void keep_attitude();

    /**
      @brief Call before copy_message.
     */
    virtual void start_copy_message();

    /**
      @brief Copy joint trajectory to inner buffer.
      this method calling in BodyRosJointControllerItem::receive_message.
      @param[in] msg Pointer of joint trajectory.
      @param[in] dst Pointer of inner buffer.
      @param[in] idx Index of points of currently processing.
      example:
      receive message -> ... [ { positons: [ 0.025 ] }, { positions: [ 0.030 ] }, { positions: [ 0.035 ] } ] ...
      if processing 'positions: [ 0.030 ]', index is setting 1.
      @param[in] jsz Number of joints.
      @retval true Apply success.
      @retval false Apply fail.
     */
    virtual bool copy_message(
                  const trajectory_msgs::JointTrajectoryPoint* msg,
                  trajectory_msgs::JointTrajectoryPoint* dst,
                  size_t idx,
                  unsigned int jsz
                  );

    /**
      @brief Call after copy_message.
     */
    virtual void end_copy_message();
};

typedef ref_ptr<BodyRosJointControllerItem> BodyRosJointControllerItemPtr;
}

#endif  /* CNOID_ROS_PLUGIN_BODY_ROS_JOINT_CONTROLLER_ITEM_H_INCLUDED */
