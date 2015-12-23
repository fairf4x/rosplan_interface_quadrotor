//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h> 
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <hector_path_follower/hector_path_follower.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"

namespace test_quadrotor
{

class Quadrotor
{
private:
  ros::NodeHandle node_handle_;

  // knowledge interface
  ros::ServiceClient update_knowledge_client;

  // topic for publishing action feedback
  ros::Publisher feedback_publisher_;

  // subscription to receive drone height messages
  ros::Subscriber height_subscriber_;

  // topic for publishing geometry_msgs::Twist data
  ros::Publisher velocity_publisher_;
  
  // published velocity  
  geometry_msgs::Twist velocity_;

  float height_;

  // flypose stuff
  tf::TransformListener tfl_;
  pose_follower::HectorPathFollower path_follower_;

  // method to add/remove simple predicate in the knowledge database
  void oneVariablePredicate(std::string pred_name, std::string var_name, std::string var_value, const char& update){
    rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
    
    updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
    updatePredSrv.request.update_type = update;
    updatePredSrv.request.knowledge.attribute_name = pred_name;
    
    diagnostic_msgs::KeyValue pair;
    pair.key = var_name;
    pair.value = var_value;
    updatePredSrv.request.knowledge.values.push_back(pair);
    
    update_knowledge_client.call(updatePredSrv);
  }

public:
  Quadrotor(ros::NodeHandle &nh) : node_handle_(nh)
  {
    // initialize knowledge client - we will be updating KMS throuch service calls
    update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");

    // initialize velocity publisher - we will send movement commands (geometry_msgs::Twist messages) through this
    velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // feedback to the planning system will be send through this topic
    feedback_publisher_ = node_handle_.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

    height_ = -1; // should be updated in heightCallback

    // hack for flypose function
    path_follower_.initialize(&tfl_);
  }

  ~Quadrotor()
  {
    stop();
  }

  // --- ACTION DISPATCH --
  void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
  {
    ROS_INFO("Quadrotor: action recieved");
    // parse action message
    std::string actionName = msg->name;
    int action_id = msg->action_id;

    if(actionName.compare("takeoff") == 0)
    {
      takeoff(action_id);
    } else if (actionName.compare("land") == 0)
    {
      land(action_id);
    } else if (actionName.compare("flysquare") == 0)
    {
      flysquare(action_id);
    } else if (actionName.compare("flypose") == 0)
    {
      flypose();
    }
  }

  void takeoff(int action_id)
  {
    // publish feedback (enabled)
    rosplan_dispatch_msgs::ActionFeedback fb;
    fb.action_id = action_id;
    fb.status = "action enabled";
    feedback_publisher_.publish(fb);

    ros::Rate loop_rate(0.25);
    ROS_INFO("Dispatching takeoff action.");
    // publish message to take off
    velocity_ = geometry_msgs::Twist();
    velocity_.linear.z = 1.0;
    velocity_publisher_.publish(velocity_);

    loop_rate.sleep();
    // publish message to stop
    stop();
    ROS_INFO("Action takeoff dispatched.");

    // update knowledge base
    oneVariablePredicate("airborne","q","q1",rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
    oneVariablePredicate("grounded","q","q1",rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);
 
    // publish feedback (achieved)
    fb.action_id = action_id;
    fb.status = "action achieved";
    feedback_publisher_.publish(fb);
  }

  void land(int action_id)
  {
    // publish feedback (enabled)
    rosplan_dispatch_msgs::ActionFeedback fb;
    fb.action_id = action_id;
    fb.status = "action enabled";
    feedback_publisher_.publish(fb);

    ROS_INFO("Dispatching land action.");
    // publish message to start descent
    velocity_ = geometry_msgs::Twist();
    velocity_.linear.z = -1.0;
    velocity_publisher_.publish(velocity_);

    // wait until the drone touches the ground
    while(height_ > 0.2)
    {
      //ROS_INFO("Current height = %f", height_);
      
      // we need to repeat heightCallback to update height_
      ros::spinOnce();
    }
    
    // publish message to stop
    stop();
    // shutdown motors
    // call service /shutdown: rosservice call /shutdown "{}"

    ROS_INFO("Action land dispatched.");

    // update knowledge base
    oneVariablePredicate("finished","q","q1",rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
    oneVariablePredicate("grounded","q","q1",rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
    oneVariablePredicate("airborne","q","q1",rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);

    // publish feedback (achieved)
    fb.action_id = action_id;
    fb.status = "action achieved";
    feedback_publisher_.publish(fb);
  }

  void flysquare(int action_id){

    // publish feedback (enabled)
    rosplan_dispatch_msgs::ActionFeedback fb;
    fb.action_id = action_id;
    fb.status = "action enabled";
    feedback_publisher_.publish(fb);

    ros::Rate loop_rate(0.5);
    ROS_INFO("Dispatching flysquare action.");
    // forward
    velocity_ = geometry_msgs::Twist();
    velocity_.linear.x = 1.0;
    velocity_publisher_.publish(velocity_);
    loop_rate.sleep();

    // right
    velocity_ = geometry_msgs::Twist();
    velocity_.linear.y = 1.0;
    velocity_publisher_.publish(velocity_);
    loop_rate.sleep();

    // backward
    velocity_ = geometry_msgs::Twist();
    velocity_.linear.x = -1.0;
    velocity_publisher_.publish(velocity_);
    loop_rate.sleep();

    // left
    velocity_ = geometry_msgs::Twist();
    velocity_.linear.y = -1.0;
    velocity_publisher_.publish(velocity_);
    loop_rate.sleep();

    stop();  
    ROS_INFO("Action flysquare dispatched.");

    // update knowledge base
    oneVariablePredicate("squaredone","q","q1",rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);

    // publish feedback (achieved)
    fb.action_id = action_id;
    fb.status = "action achieved";
    feedback_publisher_.publish(fb);
  }

  void flypose()
  {
    // define pose
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = 10;
    goal_pose.pose.position.y = 10;
    goal_pose.pose.position.z = 10;

    // define orientation
    tf::Quaternion quat(tf::Vector3(0., 0., 0.), M_PI);
    goal_pose.pose.orientation.w = quat.w();
    goal_pose.pose.orientation.x = quat.x();
    goal_pose.pose.orientation.y = quat.y();
    goal_pose.pose.orientation.z = quat.z();
    
    std::vector<geometry_msgs::PoseStamped> path;
    path.push_back(goal_pose);
    path.push_back(goal_pose);

    // fly there
    path_follower_.setPlan(path);
    
    // start navigation
    ros::Rate loop_rate(10);
    ROS_INFO("starting test loop");
    while(ros::ok()) {
      geometry_msgs::Twist twist;
      path_follower_.computeVelocityCommands(twist);
      velocity_publisher_.publish(twist);
      ros::spinOnce();
      loop_rate.sleep();
    }

  }

  // process messages received from sonar
  // -> range = 0.17 if landed
  void heightCallback(const sensor_msgs::Range::ConstPtr& msg)
  {
    // update quadrotor height
    height_ = msg->range;
  }

  void stop()
  {
    if(velocity_publisher_.getNumSubscribers() > 0)
    {
      velocity_ = geometry_msgs::Twist();
      velocity_publisher_.publish(velocity_); // sending stop message
    }
  }
};

} // namespace test_quadrotor


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_quadrotor");
  ros::NodeHandle nh;

  test_quadrotor::Quadrotor tq(nh);

  ros::Subscriber action_sub = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &test_quadrotor::Quadrotor::dispatchCallback, &tq);

  ros::Subscriber height_sub = nh.subscribe("/sonar_height", 1000, &test_quadrotor::Quadrotor::heightCallback, &tq);
  
  ros::spin();

  return 0;
}
