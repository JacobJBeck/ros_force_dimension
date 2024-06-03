/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)
 *  
 *  Created by: a. whit. (nml@whit.contact)
 *  
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Interface between ROS2 and the Force Dimension SDK for haptic robots.


// Include guard.
#ifndef FORCE_DIMENSION_NODE_H_
#define FORCE_DIMENSION_NODE_H_


// Import the ROS interface.
#include "rclcpp/rclcpp.hpp"

// Import message types.
#include "messages.hpp"

// Import the parameter management message info.
#include "rcl_interfaces/msg/set_parameters_result.hpp"



/** The ForceDimension namespace.
 */
namespace force_dimension {

  /** A ROS2 node for interfacing with a Force Dimension haptic robot.
   *  
   *  Periodically emits ROS2 messages containing the sampled pose of a Force
   *  Dimension robotic manipulandum (e.g., delta.3, sigma.7, or Novint Falcon).
   *  Accepts ROS2 messages that contain an instantaneous force, or vibration, to 
   *  be applied to the manipulandum.
   */
  class Node : public rclcpp::Node {
   
   public:
    
    // Constructor.
    Node(bool, bool);
    
    // Destructor.
    ~Node();
    
    // Configures the ROS node by creating publishers, subscriptions, and 
    // initializing parameters.
    void on_configure(void);
    
    // Activates the ROS node by initializing the Force Dimension interface and 
    // the publication timer / callback.
    void on_activate(void);
    
    //
    void on_error(void);
    
    //
    void on_deactivate(void);
    
    //  This method is expected to clear all state and return the node to a 
    // functionally equivalent state as when first created.
    // transition to Unconfigured.
    void on_cleanup(void);
    
    //
    //on_shutdown
    
   private:
    
    //
    //void Log(const char *);
    void Log(std::string);
    
    // Publishes robot state feedback.
    void PublishState(void);
    
    // Publishes robot pose messages.
    void PublishPose(void);
    
    // Publishes robot button messages.
    void PublishButton(void);
    
    // Publish gripper opening distance in meters.
    void PublishGripperGap(void);
    
    // Publish gripper opening angle in radians.
    void PublishGripperAngle(void);
    
    // Publishes robot twist messages.
    void PublishTwist(void);
    
    //// Publishes robot force messages.
    //void PublishForce(void);

    // Publish thumb and index positions
    void PublishGripperThumbPosition(void);
    void PublishGripperIndexPosition(void);

    // Subscribes to ROS messages that indicate an instantaneous force/torque
    // to be applied to the robot endpoint.
    void SubscribeWrench(void);
    
    // Subscribes to ROS messages that indicate an instantaneous force to be 
    // applied to the robot endpoint and the gripper by the robot.
    void SubscribeSigma7Force(void);
    // TO-DO: Include Delta3 force subscription.
    void SubscribeDelta3Force(void);
    
    // Applies a wrench to the robotic manipulandum, as requested via ROS 
    // message.
    void wrench_callback(const WrenchMessage);

    // Applies a force message to the sigma7 robot with forces, torques and gripper forces.
    void sigma7_force_callback(const Sigma7Message);
    
    // Check whether or not the current data sample should be published.
    bool IsPublishableSample(std::string);
    
    // Set effector mass.
    double get_effector_mass(void);
    void set_effector_mass(double mass_kg = -1);
    void reset_effector_mass(void);
    
    // Enable and disable gravity compensation.
    void set_gravity_compensation(bool);
    void set_gravity_compensation();
    
    // Enable and disable forces.
    void set_enable_force(bool);
    void set_enable_force();
    // Enable expert mode to allow wrist joint torques control.
    void set_enable_expert_mode(bool);

    // Gripper control
    void set_enable_gripper(bool);
    double get_gripper_gap(void);
    double get_gripper_angle(void);
    double get_gripper_angular_velocity(void);
    double get_max_gripper_force(void);
    void set_max_gripper_force(double);
    //void get_gripper_thumb_position();
    void get_gripper_thumb_position(double&, double&, double&);
    void get_gripper_index_position(double&, double&, double&);
    double get_gripper_linear_velocity(void);


    // Parameters set callback.
    rcl_interfaces::msg::SetParametersResult 
      set_parameters_callback(const std::vector<rclcpp::Parameter> &);
   
   private:
    int device_id_;
    float publication_interval_s_;
    bool active_;
    bool use_gripper_;
    bool use_rotation_;
    bool configured_;
    int sample_number_;
    bool hardware_disabled_;
    double baseline_effector_mass_kg_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<PoseMessage>::SharedPtr pose_publisher_;
    rclcpp::Publisher<ButtonMessage>::SharedPtr button_publisher_;
    rclcpp::Publisher<GripperGapMessage>::SharedPtr gripper_gap_publisher_;
    rclcpp::Publisher<GripperAngleMessage>::SharedPtr gripper_angle_publisher_;
    rclcpp::Publisher<TwistMessage>::SharedPtr twist_publisher_;
    //rclcpp::Publisher<ForceMessage>::SharedPtr force_publisher_;
    rclcpp::Publisher<GripperThumbPositionMessage>::SharedPtr gripper_thumb_publisher_;
    rclcpp::Publisher<GripperIndexPositionMessage>::SharedPtr gripper_index_publisher_;
    rclcpp::Subscription<WrenchMessage>::SharedPtr wrench_subscription_;
    rclcpp::Subscription<Sigma7Message>::SharedPtr sigma7_force_subscription_;
    OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
  };

}

// Include guard.
#endif

