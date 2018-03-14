#ifndef _UTILS_H_STRUCTS_
#define _UTILS_H_STRUCTS_

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>

float saturate(double in, double min_val, double max_val);

geometry_msgs::Vector3 quat2rpy(const geometry_msgs::Quaternion &quat);

geometry_msgs::Quaternion rpy2quat(const geometry_msgs::Vector3 &rpy);

double getHeadingFromQuat(const geometry_msgs::Quaternion &quat);

Eigen::Matrix3d quat2rot(const geometry_msgs::Quaternion &quat);

geometry_msgs::Quaternion rot2quat(const Eigen::Matrix3d &M);

Eigen::Matrix3d rotz(double theta);

geometry_msgs::Vector3 vel_body_frame(double theta , const geometry_msgs::Vector3 &vel_world);

geometry_msgs::Point pos_body_frame(double theta , const geometry_msgs::Point &pos);


#endif //_UTILS_H_STRUCTS_