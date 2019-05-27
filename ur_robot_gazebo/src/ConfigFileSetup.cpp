//
// Created by abdollah on 27/05/19.
//

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <sstream>
#include <std_msgs/String.h>
#include "ros/ros.h"
#include <thread>

class configClass {
private:
    /// Holds the target cubes position
    static double targetPos[3] ;
    static double jointStatePos[6] ;
    static unsigned long last_topic_index ;

    std::ifstream iFile ;

    std::ofstream oFile ;

    std::string fileLocation = "/home/abdollah/Documents/config.txt" ;

    std::unique_ptr<ros::NodeHandle> roseNode ;


public:
    configClass() {

    };

    /// getters of the values
    double* getTargetPos() {
        return targetPos ;
    }

    double* getJointStatePos() {
        return jointStatePos ;
    }

    /// Function to read the file from disk

    void readConfigFile() {

        iFile.open(fileLocation);

        std::string line;

        getline(iFile, line);


        int i1 = line.find('[');
        int i2 = line.find(',');

        int i3 = line.find(',', i2 + 1);
        int i4 = line.find(']');

        int len = i2 - i1 - 1;

        int len2 = i3 - i2 - 1;

        int len3 = i4 - i3 - 1;


        targetPos[0] = std::stod(line.substr(i1 + 1, len));
        targetPos[1] = std::stod(line.substr(i2 + 1, len2));
        targetPos[2] = std::stod(line.substr(i3 + 1, len3));

        getline(iFile, line);


        i1 = line.find('[');
        i2 = line.find(',');
        i3 = line.find(',', i2 + 1);
        i4 = line.find(',', i3 + 1);
        int i5 = line.find(',', i4 + 1);
        int i6 = line.find(',', i5 + 1);
        int i7 = line.find(']');

        len = i2 - i1 - 1;
        len2 = i3 - i2 - 1;
        len3 = i4 - i3 - 1;
        int len4 = i5 - i4 - 1;
        int len5 = i6 - i5 - 1;
        int len6 = i7 - i6 - 1;


        jointStatePos[0] = std::stod(line.substr(i1 + 1, len));
        jointStatePos[1] = std::stod(line.substr(i2 + 1, len2));
        jointStatePos[2] = std::stod(line.substr(i3 + 1, len3));
        jointStatePos[3] = std::stod(line.substr(i4 + 1, len4));
        jointStatePos[4] = std::stod(line.substr(i5 + 1, len5));
        jointStatePos[5] = std::stod(line.substr(i6 + 1, len6));

        getline(iFile, line);

        i1 = line.find(':') + 2;
        last_topic_index = std::stoul(line.substr(i1));

        iFile.close();

    }

    void writeConfigFile(const double* targetPose , const double* armState , int last_topic_index_sv) {

      std::stringstream ss ;

      ss << "target_pose : " << "[" ;

      ss << *targetPose << "," << *(targetPose + 1 ) << "," << *(targetPose + 2) << "]" << "\n" ;

      oFile.open(fileLocation) ;

      oFile << ss.rdbuf() ;

      ss.clear() ;

      ss << "arm_init_pose :" << "["  ;

      ss << *(armState) << "," ;
      ss << *(armState + 1) << "," ;
      ss << *(armState + 2) << "," ;
      ss << *(armState + 3) << "," ;
      ss << *(armState + 4) << "," ;
      ss << *(armState + 5) << "]" << "\n" ;

      oFile << ss.rdbuf() ;

      ss.clear() ;

      ss << "last_topic_index :" << last_topic_index_sv << "\n" ;

      oFile << ss.rdbuf() ;

      oFile.close() ;

    }
};



using namespace std ;

int main(int argc , char** argv) {



    ros::init(argc, argv, "config_file_setup_node");

    ros::NodeHandle n ;

    ros::Publisher robot_init_position = n.advertise<std_msgs::String>("target_position_from_disk", 1);
    ros::Publisher arm_init_state = n.advertise<std_msgs::String>("arm_init_state_disk", 1);


    return 0 ;
}