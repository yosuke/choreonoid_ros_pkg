/**
  @file
  @author
 */

#include "BodyRosTorqueControllerItem.h"

using namespace cnoid;

#if 1  /* Temporary. */
/* 
  these parameters are limited. (for JVRC-1)
  want to outer setting in the future.
 */
  
static const double pgain[] = {
  50000.0, 30000.0, 30000.0, 30000.0, 30000.0,
  80000.0, 80000.0, 10000.0, 3000.0,  30000.0,
  30000.0, 80000.0, 3000.0,  30000.0, 10000.0,
  3000.0,  3000.0,  30000.0, 30000.0, 10000.0,
  3000.0,  30000.0, 3000.0,  3000.0,  3000.0,
  3000.0,  3000.0,  3000.0,  3000.0,  3000.0,
  3000.0,  3000.0,  10000.0, 3000.0,  3000.0,
  30000.0, 3000.0,  3000.0,  3000.0,  3000.0,
  3000.0,  3000.0,  3000.0,  3000.0,
  };
  
static const double dgain[] = {
  100.0, 100.0, 100.0, 100.0, 100.0,
  100.0, 100.0, 100.0, 100.0, 100.0,
  100.0, 100.0, 100.0, 100.0, 100.0,
  100.0, 100.0, 100.0, 100.0, 100.0,
  100.0, 100.0, 100.0, 100.0, 100.0,
  100.0, 100.0, 100.0, 100.0, 100.0,
  100.0, 100.0, 100.0, 100.0, 100.0,
  100.0, 100.0, 100.0, 100.0, 100.0,
  100.0, 100.0, 100.0, 100.0,
  };
  
static const double u_lower[] = {
  -20.0, -20.0, -20.0, -50.0, -20.0,
  -50.0, -20.0, -20.0, -20.0, -50.0,
  -20.0, -50.0, -20.0, -20.0, -20.0,
  -20.0, -20.0, -20.0, -20.0, -20.0,
  -20.0, -20.0, -20.0, -20.0, -20.0,
  -20.0, -20.0, -20.0, -20.0, -20.0,
  -20.0, -20.0, -20.0, -20.0, -20.0,
  -20.0, -20.0, -20.0, -20.0, -20.0,
  -20.0, -20.0, -20.0, -20.0,
  };
  
static const double u_upper[] = {
  20.0, 20.0, 20.0, 50.0, 20.0,
  50.0, 20.0, 20.0, 20.0, 50.0,
  20.0, 50.0, 20.0, 20.0, 20.0,
  20.0, 20.0, 20.0, 20.0, 20.0,
  20.0, 20.0, 20.0, 20.0, 20.0,
  20.0, 20.0, 20.0, 20.0, 20.0,
  20.0, 20.0, 20.0, 20.0, 20.0,
  20.0, 20.0, 20.0, 20.0, 20.0,
  20.0, 20.0, 20.0, 20.0,
  };
#endif  /* Temporary. */

void BodyRosTorqueControllerItem::initialize(ExtensionManager* ext) { 
  static bool initialized = false;
  int argc = 0;
  char** argv;

  if (! ros::isInitialized()) {
    ros::init(argc, argv, "choreonoid");
  }

  if (! initialized) {
    ext->itemManager().registerClass<BodyRosTorqueControllerItem>("BodyRosTorqueControllerItem");
    ext->itemManager().addCreationPanel<BodyRosTorqueControllerItem>();
    initialized = true;
  }
}

BodyRosTorqueControllerItem::BodyRosTorqueControllerItem()
{
  controllerTarget         = 0;
  control_mode_name_       = "torque_control";
  has_trajectory_          = false;
  joint_state_update_rate_ = 100.0;
}

BodyRosTorqueControllerItem::BodyRosTorqueControllerItem(const BodyRosTorqueControllerItem& org)
  : BodyRosJointControllerItem(org)
{
  controllerTarget         = 0;
  control_mode_name_       = "torque_control";
  has_trajectory_          = false;
  joint_state_update_rate_ = 100.0;
}

BodyRosTorqueControllerItem::~BodyRosTorqueControllerItem()
{
  stop();
}

Item* BodyRosTorqueControllerItem::doDuplicate() const
{
  return new BodyRosTorqueControllerItem(*this);
}

bool BodyRosTorqueControllerItem::hook_of_start()
{
#if (DEBUG_ROS_JOINT_CONTROLLER > 0)
  ROS_DEBUG("%s: Called.", __PRETTY_FUNCTION__);
#endif  /* DEBUG_ROS_JOINT_CONTROLLER */

#if 1  /* Temporary. */
  size_t i;

  i = sizeof(pgain) / sizeof(double);

  if (i != body()->numJoints()) {
    ROS_ERROR("Size mismatch. (pgain %d joint %d)", i, body()->numJoints());
    return false;
  }

  i = sizeof(dgain) / sizeof(double);

  if (i != body()->numJoints()) {
    ROS_ERROR("Size mismatch. (dgain %d joint %d)", i, body()->numJoints());
    return false;
  }

  i = sizeof(u_lower) / sizeof(double);

  if (i != body()->numJoints()) {
    ROS_ERROR("Size mismatch. (u_lower %d joint %d)", i, body()->numJoints());
    return false;
  }

  i = sizeof(u_upper) / sizeof(double);

  if (i != body()->numJoints()) {
    ROS_ERROR("Size mismatch. (u_upper %d joint %d)", i, body()->numJoints());
    return false;
  }
#endif  /* Temporary. */

  qref_old_.resize(body()->numJoints());
  q_old_.resize(body()->numJoints());

  for (size_t i = 0; i < body()->numJoints(); i++) {
    qref_old_[i] = body()->joint(i)->q();
  }

  q_old_ = qref_old_;

  return true;
}

void BodyRosTorqueControllerItem::pd_control(Link* joint, double q_ref)
{
  double q;
  double dq_ref;
  double dq;
  double u;
  size_t i;

  if (! joint) {
    ROS_ERROR("%s: Invalid arguments", __PRETTY_FUNCTION__);
    return;
  }

  ROS_DEBUG("----- %s (joint id %d) -----", joint->name().c_str(), joint->jointId());

  i = joint->jointId();
  q = joint->q();

  if (isnan(q_ref)) {
    ROS_ERROR("Joint angle setting is NaN (%s)", joint->name().c_str());
    goto done;
  } else if (q_ref < joint->q_lower() || q_ref > joint->q_upper()) {
    ROS_ERROR("Joint angle setting is over limit (%s: lower %f upper %f set %f)",
              joint->name().c_str(), joint->q_lower(), joint->q_upper(), q_ref);
    goto done;
  } 

  dq_ref = (q_ref - qref_old_[i]) / timeStep_;

  if (isnan(dq_ref)) {
    ROS_ERROR("Calculate dq_ref, result is NaN (%s)", joint->name().c_str());
    goto done;
  } else if (dq_ref < joint->dq_lower()) {
    ROS_DEBUG("Calculate dq_ref, result is over lower limt. (adjust %f -> %f)", dq_ref, joint->dq_lower());
    dq_ref = joint->dq_lower();
  } else if (dq_ref > joint->dq_upper()) {
    ROS_DEBUG("Calculate dq_ref, result is over upper limt. (adjust %f -> %f)", dq_ref, joint->dq_upper());
    dq_ref = joint->dq_upper();
  }

  dq = (q - q_old_[i]) / timeStep_;

  if (isnan(dq)) {
    ROS_ERROR("Calculate dq, result is NaN (%s)", joint->name().c_str());
    goto done;
  } else if (dq < joint->dq_lower()) {
    ROS_DEBUG("Calculate dq, result is over lower limt. (adjust %f -> %f)", dq, joint->dq_lower());
    dq = joint->dq_lower();
  } else if (dq > joint->dq_upper()) {
    ROS_DEBUG("Calculate dq, result is over upper limt. (adjust %f -> %f)", dq, joint->dq_upper());
    dq = joint->dq_upper();
  }

  u = (q_ref - q) * pgain[i] / 100.0 + (dq_ref - dq) * dgain[i] / 100.0;

  if (! isnan(u)) {
    if (u < u_lower[i]) {
      ROS_DEBUG("Calculate u, result is over lower limt. (adjust %f -> %f)", u, u_lower[i]);
      u = u_lower[i];
    } else if (u > u_upper[i]) {
      ROS_DEBUG("Calculate u, result is over upper limt. (adjust %f -> %f)", u, u_upper[i]);
      u = u_upper[i];
    }

    ROS_DEBUG("time step %f", timeStep_);
    ROS_DEBUG("dq_lower %f dq_upper %f", joint->dq_lower(), joint->dq_upper());
    ROS_DEBUG("pgain %f dgain %f", pgain[i], dgain[i]);
    ROS_DEBUG("qref_old_ %f q_old_ %f", qref_old_[i], q_old_[i]);
    ROS_DEBUG("q_ref %f q %f dq_ref %f dq %f u %f", q_ref, q, dq_ref, dq, u);

    joint->u() = u;
    qref_old_[i] = q_ref;
  } else {
    ROS_ERROR("Calculate u, result is NaN (%s)", joint->name().c_str());
  }

 done:
  q_old_[i] = q;

  return;
}

void BodyRosTorqueControllerItem::keep_attitude()
{
  for (size_t i = 0; i < body()->numJoints(); i++) {
    pd_control(body()->joint(i), qref_old_[i]);
  }

  return;
}

void BodyRosTorqueControllerItem::apply_message(Link* joint, size_t idx, trajectory_msgs::JointTrajectoryPoint* point)
{
  double u;

  if (! joint || ! point) {
    ROS_ERROR("%s: Invalid arguments", __PRETTY_FUNCTION__);
    return;
  }

  if (point->effort.size() < 1) {
    pd_control(joint, point->positions[idx]);
  } else {
    u = point->effort[idx];

    if (! isnan(u)) {
      joint->u() = u;
    } else {
      ROS_ERROR("Effort setting is NaN (%s)", joint->name().c_str());
    }
  }

  return;
}

bool BodyRosTorqueControllerItem::copy_message(
      const trajectory_msgs::JointTrajectoryPoint* msg,
      trajectory_msgs::JointTrajectoryPoint* dst,
      size_t idx,
      unsigned int jsz
      )
{
  bool has_effort;

  if (! msg || ! dst || jsz < 1) {
    ROS_ERROR("%s: Invalid arguments", __PRETTY_FUNCTION__);
    return false;
  }

  if (msg->velocities.size() > 0 || msg->accelerations.size() > 0) {
    ROS_WARN("Can not setting velocities and accelerations in this torque controller");
    return false;
  }

  dst->positions.resize(jsz);

  if (msg->effort.size() > 0) {
    dst->effort.resize(jsz);
    has_effort = true;
  } else {
    dst->effort.resize(0);
    has_effort = false;
  }

  for (size_t i = 0; i < jsz; i++) {
    dst->positions[i] = (i < msg->positions.size()) ? msg->positions[i] : 0.0;

    if (has_effort) {
      dst->effort[i] = (i < msg->effort.size()) ? msg->effort[i] : 0.0;
    }
  }

  dst->time_from_start = ros::Duration(msg->time_from_start.sec, msg->time_from_start.nsec);

  return true;
}

