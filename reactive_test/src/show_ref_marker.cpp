/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <qcontrol_defs/AttPVA.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <fstream>

using namespace std;

double Z = 0;
bool receive_sp_sys =false;
bool receive_sp_env =false;
geometry_msgs::Point quad9_pos , quad10_pos, quad9_sp, quad10_sp;
//visualization_msgs::Marker line_q9 , line_q10 , line_q9_sp , line_q10_sp;

void save_quad9_vicon(const geometry_msgs::TransformStamped::ConstPtr& msg){
  quad9_pos.x = msg->transform.translation.x;
  quad9_pos.y = msg->transform.translation.y;
  //quad9_pos.z = msg->transform.translation.z;
  quad9_pos.z = Z;
  //line_q9.points.push_back(quad9_pos);
  //line_q9.header.stamp  = ros::Time::now();
  //sys_pos.publish(line_q9);
}
void save_quad10_vicon(const geometry_msgs::TransformStamped::ConstPtr& msg){
  quad10_pos.x = msg->transform.translation.x;
  quad10_pos.y = msg->transform.translation.y;
  //quad10_pos.z = msg->transform.translation.z;
  quad10_pos.z = Z;
  //line_q10.points.push_back(quad10_pos);
  //line_q10.header.stamp = ros::Time::now();
  //env_pos.publish(line_q10);
}
void save_quad9_setpoint(const qcontrol_defs::AttPVA::ConstPtr& msg){
  quad9_sp.x = msg->posX_roll;
  quad9_sp.y = msg->posY_pitch;
  //quad9_sp.z = msg->posZ_thrust;
  quad9_sp.z = Z;
  receive_sp_sys = true;
  //line_q9_sp.points.push_back(quad9_sp);
  //line_q9_sp.header.stamp =  ros::Time::now();
  //sys_sp.publish(line_q9_sp);
}
void save_quad10_setpoint(const qcontrol_defs::AttPVA::ConstPtr& msg){
  quad10_sp.x = msg->posX_roll;
  quad10_sp.y = msg->posY_pitch;
  //quad10_sp.z = msg->posZ_thrust;
  quad10_sp.z = Z;
  receive_sp_env = true;
  //line_q10_sp.points.push_back(quad10_sp);
  //line_q10_sp.header.stamp = ros::Time::now();
  //env_sp.publish(line_q10_sp);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
/*  ros::Publisher env_pos = n.advertise<visualization_msgs::Marker>("env_pos_marker", 10);
  ros::Publisher env_sp = n.advertise<visualization_msgs::Marker>("env_sp_marker", 10);
  ros::Publisher sys_pos = n.advertise<visualization_msgs::Marker>("sys_pos_marker", 10);
  ros::Publisher sys_sp = n.advertise<visualization_msgs::Marker>("sys_sp_marker", 10);*/

  ros::Subscriber quad9_vicon = n.subscribe<geometry_msgs::TransformStamped>("/vicon/Quad8/Quad8", 10 , save_quad9_vicon);
  ros::Subscriber quad10_vicon = n.subscribe<geometry_msgs::TransformStamped>("/vicon/Quad9/Quad9", 10 , save_quad10_vicon);
  ros::Subscriber quad9_pva = n.subscribe<qcontrol_defs::AttPVA>("/Quad8/qcontrol/att_pva", 10 , save_quad9_setpoint);
  ros::Subscriber quad10_pva = n.subscribe<qcontrol_defs::AttPVA>("/Quad9/qcontrol/att_pva", 10,save_quad10_setpoint);
/*
  line_q9.header.frame_id = line_q10.header.frame_id = line_q9_sp.header.frame_id = line_q10_sp.header.frame_id =  "1";
  line_q9.ns = line_q10.ns = line_q9_sp.ns = line_q10_sp.ns =  "lines";
  line_q9.action = line_q10.action = line_q9_sp.action = line_q10_sp.action = visualization_msgs::Marker::MODIFY;
  line_q9.pose.orientation.w = line_q10.pose.orientation.w = line_q9_sp.pose.orientation.w = line_q10_sp.pose.orientation.w = 1.0;

  line_q9.type = line_q10.type = line_q9_sp.type = line_q10_sp.type=visualization_msgs::Marker::POINTS;
  line_q9.scale.x = line_q10.scale.x = line_q9_sp.scale.x = line_q10_sp.scale.x = 0.05;
  line_q9.scale.y = line_q10.scale.y = line_q9_sp.scale.y = line_q10_sp.scale.y = 0.05;

  line_q10.color.r = 1.0;
  line_q10.color.b = 1.0;
  line_q10.color.g = 1.0;
  line_q10.color.a = 1.0;
  line_q10_sp.color.g = 1.0;
  line_q10_sp.color.a = 1.0;

  line_q9.color.r = 1.0;
  line_q9.color.a = 1.0;
  line_q9_sp.color.b = 1.0;
  line_q9_sp.color.a = 1.0;*/

  ros::Rate r(50);
  ofstream myfile, mydata;
  myfile.open ("data_plot.txt",ios::trunc);
  myfile.close();

  for (int i =0 ; i< 50 ; i++){
    ros::spinOnce();
    r.sleep();
  }

  while (ros::ok())
  {
    ros::spinOnce();
    if(receive_sp_sys && receive_sp_env){
      myfile.open("data_plot.txt",ios::app);
      myfile << quad9_pos.x << "," << quad9_pos.y <<"," << quad10_pos.x <<"," <<quad10_pos.y <<","<< quad9_sp.x << "," << quad9_sp.y <<"," << quad10_sp.x <<","<<quad10_sp.y <<"\n";
      myfile.close();
    }
    r.sleep();
  }
}
// %EndTag(FULLTEXT)%

