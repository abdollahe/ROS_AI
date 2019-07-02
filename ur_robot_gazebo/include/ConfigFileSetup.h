//
// Created by abdollah on 28/05/19.
//



#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <sstream>
#include <std_msgs/String.h>
#include "ros/ros.h"
#include <thread>
#include "../include/SystemConfig.h"

#ifndef SRC_CONFIGCLASS_H
#define SRC_CONFIGCLASS_H


using namespace std ;

class ConfigClass {

private:
    /// Holds the target cubes position
    int targetPos[4] ;
    int jointStatePos[6] ;
    unsigned long last_topic_index ;


    
    /// Input stream for reading the config file
    std::ifstream iFile ;

    /// Output stream to write the config file
    std::ofstream oFile ;

    /// Location of the config file
    std::string fileLocation = "/home/abdollah/Documents/config.txt" ;


public:
    ConfigClass() = default ;

    ~ConfigClass() = default ;

    int* getTargetPos() ;

    int* getJointStatePos() ;

    unsigned long getLastTopicIndex() ;

    void readConfigFile() ;

    void writeConfigFile(const int* targetPose , const int* armState , int last_topic_index_sv) ;


};


#endif //SRC_CONFIGCLASS_H
