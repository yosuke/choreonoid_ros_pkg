from cnoid.Base import *
from cnoid.BodyPlugin import *
from cnoid.RosPlugin import *

world = RootItem.instance().findItem('World')
worldros = WorldRosItem()
worldros.setName('WorldRos')
world.addChildItem(worldros)
#sim = SimulatorItem.findActiveSimulatorItemFor(world)
sim = RootItem.instance().findItem('AISTSimulator')
sim.setDynamicsMode(sim.DynamicsMode.HG_DYNAMICS)
visionsim = GLVisionSimulatorItem()
sim.addChildItem(visionsim)
