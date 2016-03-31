/**
  @file
  @author
 */

#include "BodyRosJointControllerItem.h"

using namespace cnoid;

BodyRosJointControllerItem::BodyRosJointControllerItem()
  : os(MessageView::instance()->cout())
{
  controllerTarget         = 0;
  control_mode_name_       = "";
  has_trajectory_          = false;
  joint_state_update_rate_ = 100.0;
}

BodyRosJointControllerItem::BodyRosJointControllerItem(const BodyRosJointControllerItem& org)
  : ControllerItem(org),
    os(MessageView::instance()->cout())
{
  controllerTarget         = 0;
  control_mode_name_       = "";
  has_trajectory_          = false;
  joint_state_update_rate_ = 100.0;
}

BodyRosJointControllerItem::~BodyRosJointControllerItem()
{
  stop();
}

Item* BodyRosJointControllerItem::doDuplicate() const
{
  return new BodyRosJointControllerItem(*this);
}

void BodyRosJointControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
  putProperty.decimals(2).min(0.0)("Update rate", joint_state_update_rate_, changeProperty(joint_state_update_rate_));
}

bool BodyRosJointControllerItem::store(Archive& archive)
{
  archive.write("jointStateUpdateRate", joint_state_update_rate_);

  return true;
}

bool BodyRosJointControllerItem::restore(const Archive& archive)
{
  archive.read("jointStateUpdateRate", joint_state_update_rate_);

  return true;
}

bool BodyRosJointControllerItem::hook_of_start()
{
#if (DEBUG_ROS_JOINT_CONTROLLER > 0)
  ROS_DEBUG("%s: Called", __PRETTY_FUNCTION__);
#endif  /* DEBUG_ROS_JOINT_CONTROLLER */

  return true;
}

bool BodyRosJointControllerItem::start(Target* target)
{
  std::string topic_name;

  if (! target) {
    MessageView::instance()->putln(MessageView::ERROR, boost::format("Target not found"));
    return false;
  } else if (! target->body()) {
    MessageView::instance()->putln(MessageView::ERROR, boost::format("BodyItem not found"));
    return false;
  } else if (control_mode_name_.empty()) {
    ROS_ERROR("%s: control_mode_name_ is empty, please report to developer", __PRETTY_FUNCTION__);
    return false;
  } else {
    std::replace(control_mode_name_.begin(), control_mode_name_.end(), '-', '_');
  }

  controllerTarget = target;
  simulationBody   = target->body();
  timeStep_        = target->worldTimeStep();
  controlTime_     = target->currentTime();

  // buffer of preserve currently state of joints.
  joint_state_.header.stamp.fromSec(controlTime_);
  joint_state_.name.resize(body()->numJoints());
  joint_state_.position.resize(body()->numJoints());
  joint_state_.velocity.resize(body()->numJoints());
  joint_state_.effort.resize(body()->numJoints());

  // preserve initial state of joints.
  for (size_t i = 0; i < body()->numJoints(); i++) {
    Link* joint = body()->joint(i);

    joint_number_map_[ joint->name() ] = i;
    joint_state_.name[i]               = joint->name();
    joint_state_.position[i]           = joint->q();
    joint_state_.velocity[i]           = joint->dq();
    joint_state_.effort[i]             = joint->u();
  }

  std::string name = body()->name();
  std::replace(name.begin(), name.end(), '-', '_');

  if (hook_of_start() == false) {
    return false;
  }

  rosnode_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(name));

  topic_name                 = control_mode_name_ + "/joint_states";
  joint_state_publisher_     = rosnode_->advertise<sensor_msgs::JointState>(topic_name, 1000);
  topic_name                 = control_mode_name_ + "/set_joint_trajectory";
  joint_state_subscriber_    = rosnode_->subscribe(
                                          topic_name, 1000, &BodyRosJointControllerItem::receive_message, this);
  joint_state_update_period_ = 1.0 / joint_state_update_rate_;
  joint_state_last_update_   = controllerTarget->currentTime();

  ROS_DEBUG("Joint state update rate %f", joint_state_update_rate_);

  async_ros_spin_.reset(new ros::AsyncSpinner(0));
  async_ros_spin_->start();

  return true;
}

void BodyRosJointControllerItem::apply_message(Link* joint, size_t idx, trajectory_msgs::JointTrajectoryPoint* point)
{
#if (DEBUG_ROS_JOINT_CONTROLLER > 0)
  ROS_DEBUG("%s: Called", __PRETTY_FUNCTION__);
#endif  /* DEBUG_ROS_JOINT_CONTROLLER */

  return;
}

void BodyRosJointControllerItem::keep_attitude()
{
#if (DEBUG_ROS_JOINT_CONTROLLER > 0)
  ROS_DEBUG("%s: No operations", __PRETTY_FUNCTION__);
#endif  /* DEBUG_ROS_JOINT_CONTROLLER */

  return;
}

bool BodyRosJointControllerItem::control()
{
  controlTime_ = controllerTarget->currentTime();
  double updateSince = controlTime_ - joint_state_last_update_;

  if (updateSince > joint_state_update_period_) {
    // publish current joint states
    joint_state_.header.stamp.fromSec(controlTime_);

    for (int i = 0; i < body()->numJoints(); i++) {
      Link* joint = body()->joint(i);
      joint_state_.position[i] = joint->q();
      joint_state_.velocity[i] = joint->dq();
      joint_state_.effort[i] = joint->u();
    }

    joint_state_publisher_.publish(joint_state_);
    joint_state_last_update_ += joint_state_update_period_;
  }

  // apply joint force based on the trajectory message
  if (has_trajectory_ && controlTime_ >= trajectory_start_) {
    if (trajectory_index_ < points_.size()) {
      unsigned int joint_size = joint_names_.size();

      for (size_t i = 0; i < joint_size; ++i) {
        std::map<std::string, int>::const_iterator it = joint_number_map_.find(joint_names_[i]);

        if (it != joint_number_map_.end()) {
          apply_message(body()->joint((*it).second), i, &points_[ trajectory_index_ ]);
        } else {
          ROS_WARN("Unknown joint name: %s", joint_names_[i].c_str());
        }
      }

      ros::Duration duration(points_[trajectory_index_].time_from_start.sec,
                             points_[trajectory_index_].time_from_start.nsec);
      trajectory_start_ += duration.toSec();
      trajectory_index_++;
    } else {
      has_trajectory_ = false;
    }
  }

  keep_attitude();

  return true;
}

void BodyRosJointControllerItem::input()
{
  return;
}

void BodyRosJointControllerItem::output()
{
  return;
}

void BodyRosJointControllerItem::stop()
{
  if (ros::ok()) {
    if (async_ros_spin_) {
      async_ros_spin_->stop();
    }

    if (rosnode_) {
      rosnode_->shutdown();
    }
  }

  return;
}

void BodyRosJointControllerItem::start_copy_message()
{
#if (DEBUG_ROS_JOINT_CONTROLLER > 0)
  ROS_DEBUG("%s: Called", __PRETTY_FUNCTION__);
#endif  /* DEBUG_ROS_JOINT_CONTROLLER */

  return;
}

void BodyRosJointControllerItem::end_copy_message()
{
#if (DEBUG_ROS_JOINT_CONTROLLER > 0)
  ROS_DEBUG("%s: Called", __PRETTY_FUNCTION__);
#endif  /* DEBUG_ROS_JOINT_CONTROLLER */

  return;
}

bool BodyRosJointControllerItem::copy_message(
      const trajectory_msgs::JointTrajectoryPoint* msg,
      trajectory_msgs::JointTrajectoryPoint* dst,
      size_t idx,
      unsigned int jsz
      )
{
#if (DEBUG_ROS_JOINT_CONTROLLER > 0)
  ROS_DEBUG("%s: No operations", __PRETTY_FUNCTION__);
#endif  /* DEBUG_ROS_JOINT_CONTROLLER */

  return false;
}

void BodyRosJointControllerItem::receive_message(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
  // copy all the trajectory info to the buffer
  unsigned int joint_size = msg->joint_names.size();

  // just in case.
  if (joint_size < 1) {
    return;
  }

  joint_names_.resize(joint_size);
  for (size_t i = 0; i < joint_size; ++i) {
    joint_names_[i] = msg->joint_names[i];
  }
  unsigned int point_size = msg->points.size();

  // just in case.
  if (point_size < 1) {
    joint_names_.resize(0);
    return;
  }

  points_.resize(point_size);

  start_copy_message();

  for (size_t i = 0; i < point_size; ++i) {
    if (! copy_message(&msg->points[i], &points_[i], i, joint_size)) {
      return;
    }
  }

  trajectory_start_ = ros::Time(msg->header.stamp.sec, msg->header.stamp.nsec).toSec();

  if (trajectory_start_ < controlTime_) {
    trajectory_start_ = controlTime_;
  }

  trajectory_index_ = 0;
  has_trajectory_ = true;

  end_copy_message();

  return;
}

