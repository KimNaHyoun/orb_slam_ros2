#pragma once

#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <chrono>
#include <thread>

class OrbSlamRos2: public rclcpp::Node{
public:
    OrbSlamRos2();
    ~OrbSlamRos2();

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr         str_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr      str_subscriber_;
    rclcpp::TimerBase::SharedPtr                                timer_;
    std::thread                                                 orb_slam_th_;

    void strSubCallback(std_msgs::msg::String str_msg);
    void timerCallback();
    void run();
    void loadImages(const std::string &strSequence, std::vector<std::string> &vstrImageFilenames,
                std::vector<double> &vTimestamps);

};