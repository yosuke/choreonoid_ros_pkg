#include "WorldRosItem.h"
#include <cnoid/BodyItem>
#include <cnoid/RootItem>
#include <cnoid/Link>
#include <cnoid/Sensor>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/ItemTreeView>

using namespace cnoid;

void WorldRosItem::initialize(ExtensionManager* ext) { 
  static bool initialized = false;
  int argc = 0;
  char** argv;
  if (!ros::isInitialized())
    ros::init(argc, argv, "choreonoid");
  if (!initialized) {
    ext->itemManager().registerClass<WorldRosItem>("WorldRosItem");
    ext->itemManager().addCreationPanel<WorldRosItem>();
    initialized = true;
  }
}

WorldRosItem::WorldRosItem()
{
  RootItem::instance()->sigTreeChanged().connect(
                                                 boost::bind(&WorldRosItem::start, this));
  start();
}

WorldRosItem::WorldRosItem(const WorldRosItem& org)
  : Item(org)
{
  RootItem::instance()->sigTreeChanged().connect(
                                                 boost::bind(&WorldRosItem::start, this));
  start();
}

WorldRosItem::~WorldRosItem()
{
  stop();
}

ItemPtr WorldRosItem::doDuplicate() const
{
  return new WorldRosItem(*this);
}

void WorldRosItem::start()
{
  static bool initialized = false;
  if (initialized) return;
  
  world = this->findOwnerItem<WorldItem>();
  if (world)
    ROS_INFO("Found WorldItem: %s", world->name().c_str());
  sim = SimulatorItem::findActiveSimulatorItemFor(this);
  if (sim == 0) return;
  
  std::string name = sim->name();
  ROS_INFO("Found SimulatorItem: %s", name.c_str());
  std::replace(name.begin(), name.end(), '-', '_');
  rosnode_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(name));

  pub_clock_ = rosnode_->advertise<rosgraph_msgs::Clock>("/clock", 10);
  pub_link_states_ = rosnode_->advertise<gazebo_msgs::LinkStates>("link_states", 10);
  pub_model_states_ = rosnode_->advertise<gazebo_msgs::ModelStates>("model_states", 10);
  TimeBar* timeBar = TimeBar::instance();
  timeBar->sigTimeChanged().connect(boost::bind(&WorldRosItem::timeTick, this, _1));
  
  std::string pause_physics_service_name("pause_physics");
  ros::AdvertiseServiceOptions pause_physics_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          pause_physics_service_name,
                                                          boost::bind(&WorldRosItem::pausePhysics,this,_1,_2),
                                                          ros::VoidPtr(), &rosqueue_);
  pause_physics_service_ = rosnode_->advertiseService(pause_physics_aso);

  std::string unpause_physics_service_name("unpause_physics");
  ros::AdvertiseServiceOptions unpause_physics_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          unpause_physics_service_name,
                                                          boost::bind(&WorldRosItem::unpausePhysics,this,_1,_2),
                                                          ros::VoidPtr(), &rosqueue_);
  unpause_physics_service_ = rosnode_->advertiseService(unpause_physics_aso);

  std::string reset_simulation_service_name("reset_simulation");
  ros::AdvertiseServiceOptions reset_simulation_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          reset_simulation_service_name,
                                                          boost::bind(&WorldRosItem::resetSimulation,this,_1,_2),
                                                          ros::VoidPtr(), &rosqueue_);
  reset_simulation_service_ = rosnode_->advertiseService(reset_simulation_aso);

  std::string spawn_vrml_model_service_name("spawn_vrml_model");
  ros::AdvertiseServiceOptions spawn_vrml_model_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnModel>(
                                                                  spawn_vrml_model_service_name,
                                                                  boost::bind(&WorldRosItem::spawnVRMLModel,this,_1,_2),
                                                                  ros::VoidPtr(), &rosqueue_);
  spawn_vrml_model_service_ = rosnode_->advertiseService(spawn_vrml_model_aso);

  std::string delete_model_service_name("delete_model");
  ros::AdvertiseServiceOptions delete_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::DeleteModel>(
                                                                   delete_model_service_name,
                                                                   boost::bind(&WorldRosItem::deleteModel,this,_1,_2),
                                                                   ros::VoidPtr(), &rosqueue_);
  delete_model_service_ = rosnode_->advertiseService(delete_aso);

  async_ros_spin_.reset(new ros::AsyncSpinner(0));
  async_ros_spin_->start();

  rosqueue_thread_.reset(new boost::thread(&WorldRosItem::queueThread, this));

  initialized = true;
}

bool WorldRosItem::timeTick(double t)
{
  publishSimTime();
  publishLinkStates();
  publishModelStates();
  return true;
}

void WorldRosItem::publishSimTime()
{
  rosgraph_msgs::Clock ros_time_;
  ros_time_.clock.fromSec(sim->currentTime());
  pub_clock_.publish(ros_time_);
}

void WorldRosItem::publishLinkStates()
{
  gazebo_msgs::LinkStates link_states;

  Item* item = world->childItem();
  while(item) {
    BodyItem* body = boost::dynamic_pointer_cast<BodyItem>(item);
    if (body) {
      for (int i = 0; i < body->body()->numLinks(); i++) {
        Link* link = body->body()->link(i);
        link_states.name.push_back(body->name() + "::" + link->name());
        Vector3 pos = link->translation();
        Quat rot = Quat(link->rotation());
        geometry_msgs::Pose pose;
        pose.position.x = pos(0);
        pose.position.y = pos(1);
        pose.position.z = pos(2);
        pose.orientation.w = rot.w();
        pose.orientation.x = rot.x();
        pose.orientation.y = rot.y();
        pose.orientation.z = rot.z();
        link_states.pose.push_back(pose);
        /*
        twist.linear.x = linear_vel.x;
        twist.linear.y = linear_vel.y;
        twist.linear.z = linear_vel.z;
        twist.angular.x = angular_vel.x;
        twist.angular.y = angular_vel.y;
        twist.angular.z = angular_vel.z;
        link_states.twist.push_back(twist);
        */
      }
    }
    item = item->nextItem();
  }
  pub_link_states_.publish(link_states);
}

void WorldRosItem::publishModelStates()
{
  gazebo_msgs::ModelStates model_states;

  Item* item = world->childItem();
  while(item) {
    BodyItem* body = boost::dynamic_pointer_cast<BodyItem>(item);
    if (body) {
      model_states.name.push_back(body->name());
      Link* link = body->body()->rootLink();
      Vector3 pos = link->translation();
      Quat rot = Quat(link->rotation());
      geometry_msgs::Pose pose;
      pose.position.x = pos(0);
      pose.position.y = pos(1);
      pose.position.z = pos(2);
      pose.orientation.w = rot.w();
      pose.orientation.x = rot.x();
      pose.orientation.y = rot.y();
      pose.orientation.z = rot.z();
      model_states.pose.push_back(pose);
      /*
      twist.linear.x = linear_vel.x;
      twist.linear.y = linear_vel.y;
      twist.linear.z = linear_vel.z;
      twist.angular.x = angular_vel.x;
      twist.angular.y = angular_vel.y;
      twist.angular.z = angular_vel.z;
      model_states.twist.push_back(twist);
      */
    }
    item = item->nextItem();
  }
  pub_model_states_.publish(model_states);
}

void WorldRosItem::queueThread()
{
  static const double timeout = 0.001;
  while (rosnode_->ok()) {
    rosqueue_.callAvailable(ros::WallDuration(timeout));
  }
}

bool WorldRosItem::pausePhysics(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  sim->pauseSimulation();
  return true;
}

bool WorldRosItem::unpausePhysics(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  sim->restartSimulation();
  return true;
}

bool WorldRosItem::resetSimulation(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  sim->startSimulation(true);
  return true;
}

bool WorldRosItem::spawnVRMLModel(gazebo_msgs::SpawnModel::Request &req,
                                  gazebo_msgs::SpawnModel::Response &res)
{
  std::string model_name = req.model_name;
  std::string model_xml = req.model_xml;

  cnoid::Vector3 trans;
  cnoid::Quat R;
  trans(0) = req.initial_pose.position.x;
  trans(1) = req.initial_pose.position.y;
  trans(2) = req.initial_pose.position.z;
  R.w() = req.initial_pose.orientation.w;
  R.x() = req.initial_pose.orientation.x;
  R.y() = req.initial_pose.orientation.y;
  R.z() = req.initial_pose.orientation.z;

  BodyItemPtr body = new BodyItem();
  body->setName(req.model_name);
  
  const char *fname = tmpnam(NULL);
  std::ofstream ofs(fname);
  ofs << model_xml << std::endl;
  ofs.close();
  body->loadModelFile(fname);
  remove(fname);
  
  body->body()->rootLink()->setTranslation(trans);
  body->body()->rootLink()->setRotation(R.matrix());
  world->addChildItem(body);

  ItemTreeView::instance()->checkItem(body);
  
  return true;
}

bool WorldRosItem::deleteModel(gazebo_msgs::DeleteModel::Request &req,
                               gazebo_msgs::DeleteModel::Response &res)
{
  ItemPtr item = world->findItem(req.model_name);
  if (!item)
  {
    ROS_ERROR("DeleteModel: model [%s] does not exist", req.model_name.c_str());
    res.success = false;
    res.status_message = "DeleteModel: model does not exist";
    return true;
  }
  item->detachFromParentItem();
  return true;
}

void WorldRosItem::stop()
{
  rosnode_->shutdown();
}
