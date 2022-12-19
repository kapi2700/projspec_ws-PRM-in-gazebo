// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

float targetX = 0.0;
float targetY = 0.0;
float xToDrive = 0;
float zToRotate = 0;
int movements = 0;
bool goodRot = false;
bool goodPos = false;
float zSpeed = 0;
float xSpeed = 0;
int i = 0;
int maxi = 8;

float path[8][2]={
  {0, 0},
  {1.3, 2.5},
  {3.9, 3.7},
  {4.8, 3.5},
  {6.4, 4.7},
  {8, 5.1},
  {9.2, 8.6},
  {10, 10}};

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class RobotSteering : public rclcpp::Node
{
public:
  RobotSteering()
      : Node("RobotSteering")
  {
    // subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "/model/vehicle_blue/odometry", 10, std::bind(&RobotSteering::topic_callback, this, _1));
    
    subscription2_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/world/testwrld/dynamic_pose/info", 10, std::bind(&RobotSteering::topic_callback_pose, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/vehicle_blue/cmd_vel", 10);
    timer_ = this->create_wall_timer(
        20ms, std::bind(&RobotSteering::timer_callback, this));
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry &msg) const
  {
    tf2::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    float x = msg.pose.pose.position.x;
    float y = msg.pose.pose.position.y;
    float zRot = yaw;
    float xdiff = targetX - x;
    float ydiff = targetY - y;
    // if (ydiff == 0)
    // {
    //   ydiff = 0;
    // }
    float calculatedX = 0;
    float calculatedZ = 0;
    float targetzRot = atan(ydiff / xdiff);
    float distance = sqrt((targetX - x) * (targetX - x) + (targetY - y) * (targetY - y));
    calculatedZ = (targetzRot-zRot);
    calculatedX = distance;

    zToRotate = calculatedZ;
    xToDrive = calculatedX;

    if (distance > 0.01)
    {
      if (goodPos)
      {
        xSpeed = 0.0;
        zSpeed = 0.0;
      }
      else if (!goodRot)
      {
        xSpeed = 0.0;
        if (abs(zToRotate) < 0.0001)
        {
          zSpeed = 0.0;
          goodRot = true;
        }
        else
        {
          if (abs(zToRotate) < 0.1)
            zSpeed = zToRotate*2;
          else
            if(zToRotate>0)
            {
              zSpeed = zToRotate*2;
            }
            else
            {
              zSpeed = zToRotate*2;
            }
        }
      }
      else if (goodRot && !goodPos)
      {
        zSpeed = 0.0;
        if (xToDrive < 0.001)
        {
          xSpeed = 0.0;
          goodPos = true;
        }
        else
        {
          if (abs(zToRotate) > 0.1)
          {
            goodRot = false;
            goodPos = false;
          }
          else
          {
            if (xToDrive < 0.2)
            {
              xSpeed = xToDrive*2;
            }
            else
            {
              xSpeed = xToDrive*2;
            }
          }
        }
      }
    }
    else
    {
      xSpeed = 0;
      zSpeed = 0;
      // targetX = path[i][0];
      // targetY = path[i][1];
      i++;
      if(i>maxi)
      {
        xSpeed = 0;
        zSpeed = 0;
        targetX=10;
        targetY =10;
      }
      else
      {
        targetX = path[i-1][0];
        targetY = path[i-1][1];
      }
      if (distance > 0.1)
      {
        goodRot = false;
        goodPos = false;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Going to Node: %s out of %s", std::to_string(i).c_str(), std::to_string(maxi).c_str());
    RCLCPP_INFO(this->get_logger(), "Current:    x: %s, y:    %s", std::to_string(x).c_str(), std::to_string(y).c_str());
    RCLCPP_INFO(this->get_logger(), "Expected:   x: %s, y:    %s", std::to_string(targetX).c_str(), std::to_string(targetY).c_str());
    RCLCPP_INFO(this->get_logger(), "Calculated: x: %s, zRot: %s", std::to_string(calculatedX).c_str(), std::to_string(calculatedZ).c_str());
    RCLCPP_INFO(this->get_logger(), "Sent:       x: %s, zRot: %s", std::to_string(xSpeed).c_str(), std::to_string(zSpeed).c_str());
  }
  void Drive(float x, float z)
  {
    geometry_msgs::msg::Twist message;
    message.linear.x = x;
    message.angular.z = z;
    //RCLCPP_INFO(this->get_logger(), "Publishing: Done\n");
    publisher_->publish(message);
  }
  void timer_callback()
  {
    Drive(xSpeed, zSpeed);
    if (xSpeed > 0)
    {
      movements++;
    }
  }

  void topic_callback_pose(const tf2_msgs::msg::TFMessage &msg) const
  {
    geometry_msgs::msg::TransformStamped transform=msg.transforms[0];
    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    float x = transform.transform.translation.x;
    float y = transform.transform.translation.y;
    float zRot = yaw;
    float xdiff = targetX - x;
    float ydiff = targetY - y;
    // if (ydiff == 0)
    // {
    //   ydiff = 0;
    // }
    float calculatedX = 0;
    float calculatedZ = 0;
    float targetzRot = atan(ydiff / xdiff);
    float distance = sqrt((targetX - x) * (targetX - x) + (targetY - y) * (targetY - y));
    calculatedZ = (targetzRot-zRot);
    calculatedX = distance;

    zToRotate = calculatedZ;
    xToDrive = calculatedX;

    if (distance > 0.01)
    {
      if (goodPos)
      {
        xSpeed = 0.0;
        zSpeed = 0.0;
      }
      else if (!goodRot)
      {
        xSpeed = 0.0;
        if (abs(zToRotate) < 0.0001)
        {
          zSpeed = 0.0;
          goodRot = true;
        }
        else
        {
          if (abs(zToRotate) < 0.1)
            zSpeed = zToRotate*2;
          else
            if(zToRotate>0)
            {
              zSpeed = zToRotate*2;
            }
            else
            {
              zSpeed = zToRotate*2;
            }
        }
      }
      else if (goodRot && !goodPos)
      {
        zSpeed = 0.0;
        if (xToDrive < 0.001)
        {
          xSpeed = 0.0;
          goodPos = true;
        }
        else
        {
          if (abs(zToRotate) > 0.1)
          {
            goodRot = false;
            goodPos = false;
          }
          else
          {
            if (xToDrive < 0.2)
            {
              xSpeed = xToDrive/4;
            }
            else
            {
              xSpeed = xToDrive/4;
            }
          }
        }
      }
    }
    else
    {
      xSpeed = 0;
      zSpeed = 0;
      // targetX = path[i][0];
      // targetY = path[i][1];
      i++;
      if(i>maxi)
      {
        xSpeed = 0;
        zSpeed = 0;
        targetX=10;
        targetY =10;
      }
      else
      {
        targetX = path[i-1][0];
        targetY = path[i-1][1];
      }
      if (distance > 0.1)
      {
        goodRot = false;
        goodPos = false;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Going to Node: %s out of %s", std::to_string(i).c_str(), std::to_string(maxi).c_str());
    RCLCPP_INFO(this->get_logger(), "Current:    x: %s, y:    %s", std::to_string(x).c_str(), std::to_string(y).c_str());
    RCLCPP_INFO(this->get_logger(), "Expected:   x: %s, y:    %s", std::to_string(targetX).c_str(), std::to_string(targetY).c_str());
    RCLCPP_INFO(this->get_logger(), "Calculated: x: %s, zRot: %s", std::to_string(calculatedX).c_str(), std::to_string(calculatedZ).c_str());
    RCLCPP_INFO(this->get_logger(), "Sent:       x: %s, zRot: %s", std::to_string(xSpeed).c_str(), std::to_string(zSpeed).c_str());
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription2_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotSteering>());
  rclcpp::shutdown();
  return 0;
}
