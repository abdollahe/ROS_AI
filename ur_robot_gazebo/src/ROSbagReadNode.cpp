#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "geometry_msgs/PoseStamped.h"

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


int main() {
    rosbag::Bag bag;
    bag.open("/home/abdollah/Documents/ROSBAGs/targetPose.bag", rosbag::bagmode::Read);

    std::vector<geometry_msgs::PoseStamped> data ;

    std::vector<std::string> topics;
    topics.push_back(std::string("target_object_pose_5"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        geometry_msgs::PoseStampedPtr s = m.instantiate<geometry_msgs::PoseStamped>();
        if (s != NULL) {
            data.push_back(*s) ;
            std::cout << "---------------------------" << std::endl;
            std::cout << s->header << std::endl ;
            std::cout << s->pose.position << std::endl ;
            std::cout << s->pose.orientation << std::endl ;
        }

    }

    std::cout << "Number of elements in the vector: " << data.size()  << std::endl ;

    bag.close();

    return 0 ;
}



