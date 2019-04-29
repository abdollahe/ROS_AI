#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "geometry_msgs/PoseStamped.h"

#include <boost/foreach.hpp>
#include <std_msgs/Bool.h>
#include "../include/StampedGripperState.h"
#include "../include/StampedJointVelocity.h"
#include "../include/StampedJointPosition.h"
#include "geometry_msgs/PoseStamped.h"

#define foreach BOOST_FOREACH


int main() {
    rosbag::Bag bag;
    bag.open("/home/abdollah/Documents/ROSBAGs/armPose.bag", rosbag::bagmode::Read);
    //bag.open("/home/abdollah/Documents/ROSBAGs/magneticGripperState.bag", rosbag::bagmode::Read);

    //std::vector<geometry_msgs::PoseStamped> data ;
    //std::vector<ur_robot_gazebo::StampedGripperState> data ;
    std::vector<geometry_msgs::PoseStamped> data ;

    std::vector<std::string> topics;
    topics.push_back(std::string("arm_position_topic_0"));
    //topics.push_back(std::string("magnetic_gripper_state_0")) ;

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        //geometry_msgs::PoseStampedPtr s = m.instantiate<geometry_msgs::PoseStamped>();
        geometry_msgs::PoseStampedPtr s = m.instantiate<geometry_msgs::PoseStamped>() ;
        //ur_robot_gazebo::StampedGripperStatePtr s = m.instantiate<ur_robot_gazebo::StampedGripperState>() ;
        if (s != NULL) {
            data.push_back(*s) ;
            std::cout << "---------------------------" << std::endl;
            //std::cout << s->velocity.data() << std::endl ;
//            std::cout << s->header << std::endl ;
//            std::cout << s->pose.position << std::endl ;
//            std::cout << s->pose.orientation << std::endl ;
        }

    }

    std::cout << "Number of elements in the vector: " << data.size()  << std::endl ;

    bag.close();

    return 0 ;
}



