/** Functionality related to publishing the Force Dimension gravity
 *  compensation functionality.
 */

/** Copyright 2024 Neuromechatronics Lab, Carnegie Mellon University
 *
 *  Created by: Jonathan Shulgach (jshulgac@andrew.cmu.edu)
 *
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */



// Include guard.
#ifndef FORCE_DIMENSION_GRIPPER_H_
#define FORCE_DIMENSION_GRIPPER_H_

// Import the node header.
#include "node.hpp"

// Import the ROS interface.
#include "rclcpp/rclcpp.hpp"

// Import the Force Dimension haptics library.
#include "dhdc.h"


/** Enable or disable gripper forces.
 */
void force_dimension::Node::set_enable_gripper(bool enable) {
  // Only relevant to grippers with default closed/open state, such as the Novint Falcon.
  if (dhdGetSystemType() == DHD_DEVICE_FALCON) {
    // Enable/disable gripper force mode
    int val = enable ? DHD_ON : DHD_OFF;
    int result = dhdEnableGripperForce(val);

    // Log the result
    std::string message = "Gripper Forces: ";
    if (result != 0) {
      message += "Failed to ";
      on_error();
    } else {
      message += enable ? "Enabled" : "Disabled";
    }
    message += hardware_disabled_ ? " (unknown error)" : "";
    Log(message);
  } else if (use_gripper_){
    // If device is not Novint Falcon, check if gripper is present
    auto message = enable ? "Gripper Forces: Enabled" : "Gripper Forces: Disabled";
    Log(message);
  }
}

// Read gripper gap
double force_dimension::Node::get_gripper_gap() {
  double gap = -1;
  bool has_gripper = hardware_disabled_ ? false : dhdHasGripper(device_id_);
  int result = has_gripper ? dhdGetGripperGap(&gap, device_id_) : 0;
  if((result != 0) & (result != DHD_TIMEGUARD))  {
    std::string message = "Failed to read gripper gap";
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log("Failed to read gripper gap: unknown error");
    on_error();
  }
  return gap;
}

// Read gripper angle (in radians)
double force_dimension::Node::get_gripper_angle() {
  double angle = -1;
  bool has_gripper = hardware_disabled_ ? false : dhdHasGripper(device_id_);
  int result = has_gripper ? dhdGetGripperAngleRad(&angle, device_id_) : 0;
  if(result == -1) {
    std::string message = "Failed to read gripper angle";
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log("Failed to read gripper angle: unknown error");
    on_error();
  }
  return angle;
}

// Get the gripper angular velocity in Rad/s
double force_dimension::Node::get_gripper_angular_velocity() {
  double wv;
  int result = hardware_disabled_ ? DHD_NO_ERROR : dhdGetGripperAngularVelocityRad(&wv, device_id_);
  if(result == -1) {
    std::string message = "Failed to read gripper angular velocity";
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log("Failed to read gripper angular velocity: unknown error");
    on_error();
  }
  return wv;
}

// Get the max gripper force
double force_dimension::Node::get_max_gripper_force() {
  double max_force = dhdGetMaxGripperForce(device_id_);
  if(max_force < 0) {
    std::string message = "No limit is enforced for gripper force";
    Log("Warning: no limit enforced for gripper force");
  }
  return max_force;
}

// Set the max gripper force
void force_dimension::Node::set_max_gripper_force(double max_force) {
  int result = dhdSetMaxGripperForce(max_force, device_id_);
  if(result == -1) {
    std::string message = "Failed to set max gripper force";
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log("Failed to set max gripper force: unknown error");
    on_error();
  }
  else
  {
    Log("Max gripper force set to " + std::to_string(max_force));
  }
}

// Get the gripper thumb position by passing the pointer to the x, y, and z coordinates
void force_dimension::Node::get_gripper_thumb_position(double& px, double& py, double& pz) {
  if (!use_gripper_){
    std::string message = "Cannot get thumb position: Gripper is not enabled";
    Log("Cannot get thumb position: Gripper is not enabled");
  }
  else{
    int result = hardware_disabled_ ? 0 : dhdGetGripperThumbPos(&px, &py, &pz, device_id_);
    if(result == -1) {
      std::string message = "Failed to read gripper thumb position";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log("Failed to read gripper thumb position: unknown error");
      on_error();
    }
    //std::cout << "Thumb position: " << px << " " << py << " " << pz << std::endl;
  }
}

// Get the gripper index position
void force_dimension::Node::get_gripper_index_position(double& px, double& py, double& pz) {
  if (!use_gripper_){
    std::string message = "Cannot get finger position: Gripper is not enabled";
    Log("Cannot get finger position: Gripper is not enabled");
  }
  else{
    int result = hardware_disabled_ ? 0 : dhdGetGripperFingerPos(&px, &py, &pz, device_id_);
    if(result == -1) {
      std::string message = "Failed to read gripper finger position";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log("Failed to read gripper finger position: unknown error");
      on_error();
    }
    //std::cout << "Finger position: " << px << " " << py << " " << pz << std::endl;
  }
}

// Get the gripper linear velocity in m/s
double force_dimension::Node::get_gripper_linear_velocity() {
  double lv;
  int result = hardware_disabled_ ? 0 : dhdGetGripperLinearVelocity(&lv, device_id_);
  if(result == -1) {
    std::string message = "Failed to read gripper linear velocity";
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log("Failed to read gripper linear velocity: unknown error");
    on_error();
  }
  return lv;
}


#endif // FORCE_DIMENSION_GRIPPER_H_
