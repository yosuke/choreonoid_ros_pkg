#ifndef CNOID_ROS_PLUGIN_BODY_ROS_ITEM_H_INCLUDED
#define CNOID_ROS_PLUGIN_BODY_ROS_ITEM_H_INCLUDED

#include <cnoid/ControllerItem>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/Body>
#include <cnoid/VisionSensor>
#include <cnoid/Camera>
#include "exportdecl.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Accel.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <image_transport/image_transport.h>

#include <vector>

namespace cnoid {

class CNOID_EXPORT BodyRosItem : public ControllerItem
{
public:
    static void initialize(ExtensionManager* ext);
    
    BodyRosItem();
    BodyRosItem(const BodyRosItem& org);
    virtual ~BodyRosItem();
    bool createSensors(BodyPtr body);
    
    virtual bool start(Target* target);
    virtual double timeStep() const {
      return timeStep_;
    };
    virtual void input();
    virtual bool control();
    virtual void output();
    virtual void stop();
    
    void callback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    
    const BodyPtr& body() const { return simulationBody; };
    const DeviceList<ForceSensor>& forceSensors() const { return forceSensors_; }
    const DeviceList<RateGyroSensor>& gyroSensors() const { return gyroSensors_; }
    const DeviceList<AccelSensor>& accelSensors() const { return accelSensors_; }
    const DeviceList<Camera>& visionSensors() const { return visionSensors_; }
    
    double controlTime() const { return controlTime_; }
    
    void setModuleName(const std::string& name);

protected:
    virtual ItemPtr doDuplicate() const;
        
private:
    BodyPtr simulationBody;
    DeviceList<ForceSensor> forceSensors_;
    DeviceList<RateGyroSensor> gyroSensors_;
    DeviceList<AccelSensor> accelSensors_;
    DeviceList<Camera> visionSensors_;
    double timeStep_;

    const Target* controllerTarget;
    double controlTime_;
    std::ostream& os;

    std::string bodyName;

    boost::shared_ptr<ros::NodeHandle> rosnode_;
    sensor_msgs::JointState joint_state_;
    ros::Publisher joint_state_publisher_;
    ros::Subscriber joint_state_subscriber_;
    
    std::map<std::string, int> joint_number_map_;
    std::vector<std::string> joint_names_;
    std::vector<trajectory_msgs::JointTrajectoryPoint> points_;
    double trajectory_start_;
    unsigned int trajectory_index_;
    bool has_trajectory_;
 
    std::vector<ros::Publisher> force_sensor_publishers_;
    std::vector<ros::Publisher> rate_gyro_sensor_publishers_;
    std::vector<ros::Publisher> accel_sensor_publishers_;
    std::vector<image_transport::Publisher> vision_sensor_publishers_;
};

typedef ref_ptr<BodyRosItem> BodyRosItemPtr;
}

#endif
