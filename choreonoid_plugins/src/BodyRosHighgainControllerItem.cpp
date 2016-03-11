/**
  @file
  @author
 */

#include "BodyRosHighgainControllerItem.h"

using namespace cnoid;

void BodyRosHighgainControllerItem::initialize(ExtensionManager* ext) { 
  static bool initialized = false;
  int argc = 0;
  char** argv;

  if (! ros::isInitialized()) {
    ros::init(argc, argv, "choreonoid");
  }

  if (! initialized) {
    ext->itemManager().registerClass<BodyRosHighgainControllerItem>("BodyRosHighgainControllerItem");
    ext->itemManager().addCreationPanel<BodyRosHighgainControllerItem>();
    initialized = true;
  }
}

BodyRosHighgainControllerItem::BodyRosHighgainControllerItem()
{
  controllerTarget         = 0;
  control_mode_name_       = "highgain_control";
  has_trajectory_          = false;
  joint_state_update_rate_ = 100.0;
}

BodyRosHighgainControllerItem::BodyRosHighgainControllerItem(const BodyRosHighgainControllerItem& org)
  : BodyRosJointControllerItem(org)
{
  controllerTarget         = 0;
  control_mode_name_       = "highgain_control";
  has_trajectory_          = false;
  joint_state_update_rate_ = 100.0;
}

BodyRosHighgainControllerItem::~BodyRosHighgainControllerItem()
{
  stop();
}

Item* BodyRosHighgainControllerItem::doDuplicate() const
{
  return new BodyRosHighgainControllerItem(*this);
}

bool BodyRosHighgainControllerItem::hook_of_start()
{
  timeStep2_ = timeStep_ * timeStep_;

  return true;
}

void BodyRosHighgainControllerItem::apply_message(Link* joint, size_t idx, trajectory_msgs::JointTrajectoryPoint* point)
{
  double q;
  double dq;
  double ddq;
  size_t i;

  if (! joint || ! point) {
    ROS_ERROR("%s: Invalid arguments", __PRETTY_FUNCTION__);
    return;
  }

  ROS_DEBUG("----- %s (joint id %d) -----", joint->name().c_str(), joint->jointId());

  i = joint->jointId();
  q = point->positions[idx];

  if (isnan(q)) {
    ROS_ERROR("Joint angle setting is NaN (%s)", joint->name().c_str());
    return;
  } else if (q < joint->q_lower() || q > joint->q_upper()) {
    ROS_ERROR("Joint angle setting is over limit (%s: lower %f upper %f set %f)",
              joint->name().c_str(), joint->q_lower(), joint->q_upper(), q);
    return;
  }

  if (point->velocities.size() > 0) {
    dq = point->velocities[idx];
  } else {
    dq = (q - joint->q()) / timeStep_;
  }

  if (isnan(dq)) {
    ROS_ERROR("Joint velocity setting is NaN (%s)", joint->name().c_str());
    return;
  } else if (dq < joint->dq_lower()) {
    ROS_DEBUG("Calculate dq, result is over lower limt. (adjust %f -> %f)", dq, joint->dq_lower());
    dq = joint->dq_lower();
  } else if (dq > joint->dq_upper()) {
    ROS_DEBUG("Calculate dq, result is over upper limt. (adjust %f -> %f)", dq, joint->dq_upper());
    dq = joint->dq_upper();
  }

  if (point->accelerations.size() > 0) {
    ddq = point->accelerations[idx];
  } else {
    ddq = (q - joint->q()) / timeStep2_;
  }

  if (isnan(ddq)) {
    ROS_ERROR("Joint acceleration setting is NaN (%s)", joint->name().c_str());
    return;
  } else if (ddq < joint->dq_lower()) {
    ROS_DEBUG("Calculate ddq, result is over lower limt. (adjust %f -> %f)", ddq, joint->dq_lower());
    ddq = joint->dq_lower();
  } else if (ddq > joint->dq_upper()) {
    ROS_DEBUG("Calculate ddq, result is over lower limt. (adjust %f -> %f)", ddq, joint->dq_lower());
    ddq = joint->dq_upper();
  }

  ROS_DEBUG("q %f dq %f ddq %f q_old %f", q, dq, ddq, joint->q());

  joint->q()   = q;
  joint->dq()  = dq;
  joint->ddq() = ddq;

  return;
}

bool BodyRosHighgainControllerItem::copy_message(
      const trajectory_msgs::JointTrajectoryPoint* msg,
      trajectory_msgs::JointTrajectoryPoint* dst,
      size_t idx,
      unsigned int jsz
      )
{
  bool has_vel;
  bool has_accel;

  if (! msg || ! dst || jsz < 1) {
    ROS_ERROR("%s: Invalid arguments", __PRETTY_FUNCTION__);
    return false;
  }

  if (msg->effort.size() > 0) {
    ROS_WARN("Can not setting effort in this high-gain controller");
    return false;
  }

  dst->positions.resize(jsz);

  if (msg->velocities.size() > 0) {
    dst->velocities.resize(jsz);
    has_vel = true;
  } else {
    dst->velocities.resize(0);
    has_vel = false;
  }

  if (msg->accelerations.size() > 0) {
    dst->accelerations.resize(jsz);
    has_accel = true;
  } else {
    dst->accelerations.resize(0);
    has_accel = false;
  }

  for (size_t i = 0; i < jsz; i++) {
    dst->positions[i] = (i < msg->positions.size()) ? msg->positions[i] : 0.0;

    if (has_vel) {
      dst->velocities[i] = (i < msg->velocities.size()) ? msg->velocities[i] : 0.0;
    }

    if (has_accel) {
      dst->accelerations[i] = (i < msg->accelerations.size()) ? msg->accelerations[i] : 0.0;
    }
  }

  dst->time_from_start = ros::Duration(msg->time_from_start.sec, msg->time_from_start.nsec);

  return true;
}

