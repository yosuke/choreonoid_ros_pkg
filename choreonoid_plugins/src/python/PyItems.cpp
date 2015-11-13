/*!
  @author 
*/

#include "../BodyRosItem.h"
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
  
  class_< WorldRosItem, WorldRosItemPtr, bases<Item> >("WorldRosItem")
    ;
  implicitly_convertible<WorldRosItemPtr, ItemPtr>();
  PyItemList<WorldRosItem>("WorldRosItemList");
}
