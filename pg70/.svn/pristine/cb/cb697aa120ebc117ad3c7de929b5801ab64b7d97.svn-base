#include "PG70.hpp"

#include <rw/common/PropertyMap.hpp>
#include <rw/loaders/xml/XMLPropertyLoader.hpp>
#include <boost/bind.hpp> 

  
using namespace rw::common;
using namespace rw::loaders;
 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PG70_Node");
    ros::NodeHandle nh;

    if (argc <= 1)
    {
        // Get parameters
        std::string serialName;
        nh.param<std::string>("/PG70/PG70/SerialName", serialName, "/dev/ttyUSB0");

        PG70 pg70(serialName);
        pg70.run();
	}
    else
    {
        try
        {
            PropertyMap properties = XMLPropertyLoader::load(argv[1]);
            PG70 pg70(properties);
            pg70.run();
        }
        catch (const Exception& exp)
        {
            std::cout << "Exception = " << exp.what() << std::endl;
        }
    }

    return 0;
}

