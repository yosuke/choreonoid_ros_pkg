/**
  @file
  @author
 */

#ifndef CNOID_ROS_PLUGIN_BODY_ROS_TORQUE_CONTROLLER_ITEM_H_INCLUDED
#define CNOID_ROS_PLUGIN_BODY_ROS_TORQUE_CONTROLLER_ITEM_H_INCLUDED

#include "BodyRosJointControllerItem.h"

namespace cnoid {

class CNOID_EXPORT BodyRosTorqueControllerItem : public BodyRosJointControllerItem
{
public:
    static void initialize(ExtensionManager* ext);
    
    BodyRosTorqueControllerItem();
    BodyRosTorqueControllerItem(const BodyRosTorqueControllerItem& org);
    virtual ~BodyRosTorqueControllerItem();

protected:
    virtual Item* doDuplicate() const;
    virtual bool hook_of_start();
    virtual void keep_attitude();
    virtual void apply_message(Link* joint, size_t idx, trajectory_msgs::JointTrajectoryPoint* point);
    virtual bool copy_message(
                  const trajectory_msgs::JointTrajectoryPoint* msg,
                  trajectory_msgs::JointTrajectoryPoint* dst,
                  size_t idx,
                  unsigned int jsz
                  );

private:
    std::vector<double> qref_old_;
    std::vector<double> q_old_;

    void pd_control(Link* joint, double q_ref);
};

typedef ref_ptr<BodyRosTorqueControllerItem> BodyRosTorqueControllerItemPtr;
}

#endif  /* CNOID_ROS_PLUGIN_BODY_ROS_TORQUE_CONTROLLER_ITEM_H_INCLUDED */
