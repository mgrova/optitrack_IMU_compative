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

#include <serial/serial.h>

#include <Eigen/Eigen>

#include <iostream>
#include <chrono>
#include <fstream>

int main(int _argc , char **_argv){

    if (_argc < 3){
        std::cout << "Expected port and baudrate passed by command line. \n";
        return 0;
    }

    ros::init(_argc, _argv, "OPTITRACK_IMU_COMPARATION");

    ros::NodeHandle nh;

    Eigen::Vector3f pose;
    Eigen::Quaternionf orientation;

    ros::Subscriber subPose = nh.subscribe<geometry_msgs::PoseStamped>("/optitrack/pose",   1, [&](const geometry_msgs::PoseStamped::ConstPtr& _msg){
        pose[0] = _msg->pose.position.x;
        pose[1] = _msg->pose.position.y;
        pose[2] = _msg->pose.position.z;

        orientation.w() =  _msg->pose.orientation.w;
        orientation.x() =  _msg->pose.orientation.x;
        orientation.y() =  _msg->pose.orientation.y;
        orientation.z() =  _msg->pose.orientation.z;

    });
    
    // Read IMU using serial port
    serial::Serial ser;
    std::string port = _argv[1];
    int baudrate = atoi(_argv[2]);
    try
    {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        std::cout << "Unable to open port " << "\n";
        return -1;
    }

    // Check if is sucessfully open
    if(ser.isOpen()){
        std::cout << "Serial Port initialized" << "\n";
    }else{
        return -1;
    }

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::Rate r(50); // set 100 Hz 
    
    // Open log file
    std::ofstream log;
    log.open("/home/grvc/POSITOL_measures_" + std::to_string(time(NULL)) + ".txt");

    auto t0 = std::chrono::system_clock::now();
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            std::string result = ser.readline();
            if (result != "")
                std::cout << "Read from serial port: " << result << "\n";
        }

        auto t1 = std::chrono::system_clock::now();
        float incT = float(std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count())/1000.0;
        // Save dataset
        log << std::to_string( 0.0 ) + " " + std::to_string( float(0.0) ) + " " + std::to_string( float(0.0) ) + " " + "\n";
        log.flush();

        t0 = t1;
        r.sleep();
    }



    return 0;
}