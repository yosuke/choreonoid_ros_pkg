/**
  @file
  @author
 */

#ifndef CNOID_ROS_PLUGIN_BODY_ROS_HIGHGAIN_CONTROLLER_ITEM_H_INCLUDED
#define CNOID_ROS_PLUGIN_BODY_ROS_HIGHGAIN_CONTROLLER_ITEM_H_INCLUDED

#include "BodyRosJointControllerItem.h"

namespace cnoid {

class CNOID_EXPORT BodyRosHighgainControllerItem : public BodyRosJointControllerItem
{
public:
    static void initialize(ExtensionManager* ext);

    BodyRosHighgainControllerItem();
    BodyRosHighgainControllerItem(const BodyRosHighgainControllerItem& org);
    virtual ~BodyRosHighgainControllerItem();

protected:
    virtual Item* doDuplicate() const;
    virtual bool hook_of_start();
    virtual void apply_message(Link* joint, size_t idx, trajectory_msgs::JointTrajectoryPoint* point);
    virtual bool copy_message(
                  const trajectory_msgs::JointTrajectoryPoint* msg,
                  trajectory_msgs::JointTrajectoryPoint* dst,
                  size_t idx,
                  unsigned int jsz
                  );

private:
    double timeStep2_;
};

typedef ref_ptr<BodyRosHighgainControllerItem> BodyRosHighgainControllerItemPtr;
}

#endif  /* CNOID_ROS_PLUGIN_BODY_ROS_HIGHGAIN_CONTROLLER_ITEM_H_INCLUDED */
