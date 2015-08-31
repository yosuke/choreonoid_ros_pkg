#include "BodyRosItem.h"
#include <cnoid/Plugin>

using namespace cnoid;

class RosBodyPlugin : public Plugin
{
public:
  RosBodyPlugin() : Plugin("RosBody") { }
  
  virtual bool initialize() {
    BodyRosItem::initialize(this);
    return true;
  }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(RosBodyPlugin);
