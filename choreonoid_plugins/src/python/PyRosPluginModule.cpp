/*!
  @author 
*/

#include "../BodyRosItem.h"
#include "../WorldRosItem.h"
#include <cnoid/PyUtil>

using namespace boost::python;
using namespace cnoid;

void exportItems();

BOOST_PYTHON_MODULE(RosPlugin)
{
    boost::python::import("cnoid.Base");
    boost::python::import("cnoid.Body");
    
    exportItems();
}
