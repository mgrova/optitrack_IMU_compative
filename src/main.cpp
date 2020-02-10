//---------------------------------------------------------------------------------------------------------------------
//  OPTITRACK IMU COMPARATIVE
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 - Marco Montes Grova (a.k.a. mgrova) marrcogrova@gmail.com 
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
//  and associated documentation files (the "Software"), to deal in the Software without restriction, 
//  including without limitation the rights to use, copy, modify, merge, publish, distribute, 
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial 
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES 
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <Eigen/Eigen>

#include <iostream>
#include <chrono>
#include <fstream>

int main(int _argc , char **_argv){

    ros::init(_argc, _argv, "T265_VELOCITY");

    ros::NodeHandle nh;
    ros::Publisher pubVel = nh.advertise<geometry_msgs::TwistStamped>("/velocity_estimated", 1);

    ros::Subscriber subPose = nh.subscribe<geometry_msgs::PoseStamped>("/optitrack/pose",   1, [&](const geometry_msgs::PoseStamped::ConstPtr& _msg){


    });

    ros::Subscriber subOrientation = nh.subscribe<geometry_msgs::PoseStamped>("/optitrack/pose",   1, [&](const geometry_msgs::PoseStamped::ConstPtr& _msg){


    });

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::Rate r(100); // set 100 Hz 
    
    // Open log file
    std::ofstream log;
    log.open("/home/grvc/POSITOL_measures_" + std::to_string(time(NULL)) + ".txt");

    auto t0 = std::chrono::system_clock::now();
    while(ros::ok()){

        auto t1 = std::chrono::system_clock::now();
        float incT = float(std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count())/1000.0;
        
        ros::spinOnce();
        
        // Save dataset
        log << std::to_string( 0.0 ) + " " + std::to_string( float(0.0) ) + " " + std::to_string( float(0.0) ) + " " + "\n";
        log.flush();

        t0 = t1;
        r.sleep();
    }



    return 0;
}