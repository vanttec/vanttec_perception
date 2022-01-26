#include <ros/ros.h>
#include <iostream>
#include <tf/tf.h>

int main(){

  /**< Declaration of quaternion */
  tf::Quaternion q;
  q.setW(1);
  q.setX(0);
  q.setY(0);
  q.setZ(0);

  tf::Quaternion q2(0, 0, 0, 1); // x, y, z, w in order

  /**< quaternion -> rotation Matrix */
  tf::Matrix3x3 m(q);
  
  tf::Matrix3x3 m2;
  m2.setRotation(q); 
 
  /**< rotation Matrix - > quaternion */
  m.getRotation(q);
  
  /**< rotation Matrix -> rpy */
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  /**< rpy -> quaternion */
  tf::Quaternion q3;
  q3.setRPY(roll, pitch, yaw);
  q3.normalize();

  return 0;
}