#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "geometry_msgs/PoseStamped.h"

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


int main() {
    rosbag::Bag bag;
    bag.open("/home/abdollah/Documents/ROSBAGs/armPose.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("Test Topic"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        geometry_msgs::PoseStampedPtr s = m.instantiate<geometry_msgs::PoseStamped>();
        if (s != NULL) {
            std::cout << "---------------------------" << std::endl;
            std::cout << s->header << std::endl ;
            std::cout << s->pose.position << std::endl ;
            std::cout << s->pose.orientation << std::endl ;
        }

    }

    bag.close();

    return 0 ;
}



