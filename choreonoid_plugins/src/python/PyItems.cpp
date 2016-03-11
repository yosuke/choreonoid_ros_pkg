/*!
  @author 
*/

#include "../BodyRosItem.h"
#include "../BodyRosHighgainControllerItem.h"
#include "../BodyRosTorqueControllerItem.h"
#include "../WorldRosItem.h"
#include <cnoid/BodyState>
#include <cnoid/PyBase>

using namespace boost::python;
using namespace cnoid;

void exportItems()
{
  class_< BodyRosItem, BodyRosItemPtr, bases<Item> >("BodyRosItem")
    ;
  implicitly_convertible<BodyRosItemPtr, ItemPtr>();
  PyItemList<BodyRosItem>("BodyRosItemList");

  class_< BodyRosHighgainControllerItem, BodyRosHighgainControllerItemPtr, bases<Item> >
    ("BodyRosHighgainControllerItem")
    ;
  implicitly_convertible<BodyRosHighgainControllerItemPtr, ItemPtr>();
  PyItemList<BodyRosHighgainControllerItem>("BodyRosHighgainControllerItemList");

  class_< BodyRosTorqueControllerItem, BodyRosTorqueControllerItemPtr, bases<Item> >
    ("BodyRosTorqueControllerItem")
    ;
  implicitly_convertible<BodyRosTorqueControllerItemPtr, ItemPtr>();
  PyItemList<BodyRosTorqueControllerItem>("BodyRosTorqueControllerItemList");

  class_< WorldRosItem, WorldRosItemPtr, bases<Item> >("WorldRosItem")
    ;
  implicitly_convertible<WorldRosItemPtr, ItemPtr>();
  PyItemList<WorldRosItem>("WorldRosItemList");
}
