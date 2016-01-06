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
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h> 
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "mongodb_store/message_store.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <hector_move_base/NavigateAction.h>

namespace rosplan_interface_quadrotor
{

class Quadrotor
{
private:
  ros::NodeHandle node_handle_;

  // knowledge interface
  ros::ServiceClient update_knowledge_client;

  // motor interface
  ros::ServiceClient motor_service_client;

  // topic for publishing action feedback
  ros::Publisher feedback_publisher_;

  // subscription to receive drone height messages
  ros::Subscriber height_subscriber_;

  // topic for publishing geometry_msgs::Twist data
  ros::Publisher velocity_publisher_;
  
  // published velocity  
  geometry_msgs::Twist velocity_;

  float height_;

  // database - waypoints are retrieved from here
  mongodb_store::MessageStoreProxy message_store;
  
  // action client for fly_waypoint
  actionlib::SimpleActionClient<hector_move_base::NavigateAction> action_client;

  // method to add/remove simple predicate in the knowledge database
  void oneVariablePredicate(std::string pred_name, std::string var_name, std::string var_value, const char& update)
  {
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

  // helper function for feedback publishing
  void publishFeedback(int action_id, std::string fbstat)
  {
    rosplan_dispatch_msgs::ActionFeedback fb;
    fb.action_id = action_id;
    fb.status = fbstat;
    feedback_publisher_.publish(fb);
  }

public:
  Quadrotor(ros::NodeHandle &nh, std::string &actionserver) : node_handle_(nh), message_store(nh), action_client(actionserver,true)
  {
    ROS_INFO("hector_move_base: Waiting for nav action server to start.");
    // wait for the action server to start
    action_client.waitForServer(); //will wait for infinite time

    // initialize knowledge client - we will be updating KMS throuch service calls
    update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");

    // initialize motor client - this service is used for motor shutdown
    motor_service_client = nh.serviceClient<std_srvs::Empty>("/shutdown");

    // initialize velocity publisher - we will send movement commands (geometry_msgs::Twist messages) through this
    velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // feedback to the planning system will be send through this topic
    feedback_publisher_ = node_handle_.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

    height_ = -1; // should be updated in heightCallback
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
    } else if (actionName.compare("fly_waypoint") == 0)
    {
      fly_waypoint(msg);
    }
  }

  void takeoff(int action_id)
  {
    publishFeedback(action_id,"action enabled");

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
 
    publishFeedback(action_id,"action achieved");
  }

  void land(int action_id)
  {
    publishFeedback(action_id,"action enabled");
    
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
    
    // stop drone and shutdown motors
    emergency();

    ROS_INFO("Action land dispatched.");

    // update knowledge base
    oneVariablePredicate("finished","q","q1",rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
    oneVariablePredicate("grounded","q","q1",rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
    oneVariablePredicate("airborne","q","q1",rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);

    publishFeedback(action_id,"action achieved");
  }

  void flysquare(int action_id){
    publishFeedback(action_id,"action enabled");

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

    publishFeedback(action_id,"action achieved");
  }


  void fly_waypoint(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg){
    // get action id - for feedback publishing    
    int action_id = msg->action_id;

    // get waypoint ID from action dispatch
    std::string wpID;
    bool found = false;
    for(size_t i=0; i<msg->parameters.size(); i++) {
      if(0==msg->parameters[i].key.compare("to")) {
        wpID = msg->parameters[i].value;
        found = true;
      }
    }
    if(!found) {
      ROS_INFO("ERROR: (hector_move_base) aborting action dispatch; malformed parameters");
      return;
    }

    // get pose from message store
    std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
    if(message_store.queryNamed<geometry_msgs::PoseStamped>(wpID, results)) {
      if(results.size()<1) {
        ROS_INFO("ERROR: (hector_move_base) aborting action dispatch; no matching wpID %s", wpID.c_str());
        return;
      }
      if(results.size()>1)
        ROS_INFO("ERROR: (hector_move_base) multiple waypoints share the same wpID");
      
      // dispatch MoveBase action
      hector_move_base::NavigateGoal goal;
      geometry_msgs::PoseStamped &pst = *results[0];
      goal.end.x = pst.pose.position.x;
      goal.end.y = pst.pose.position.y;

      publishFeedback(action_id,"action enabled");

      // call action server to navigate to waypoint
      action_client.sendGoalAndWait(goal);
      if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
          ROS_INFO("Navigate action is completed!");
          // update knowledge base
          /*
          oneVariablePredicate("finished","q","q1",rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
          oneVariablePredicate("grounded","q","q1",rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
          oneVariablePredicate("airborne","q","q1",rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);
          */

          publishFeedback(action_id,"action achieved");
      }
      else
      {
          publishFeedback(action_id,"action failed");
          ROS_ERROR("Navigate action failed!");
      }
    }
  }

  // process messages received from sonar
  // -> range = 0.17 if landed
  void heightCallback(const sensor_msgs::Range::ConstPtr& msg)
  {
    // update quadrotor height
    height_ = msg->range;
  }

  void emergency()
  {
    // publish message to stop
    stop();
    // shutdown motors
    shutdown();
  }

  void stop()
  {
    if(velocity_publisher_.getNumSubscribers() > 0)
    {
      velocity_ = geometry_msgs::Twist();
      velocity_publisher_.publish(velocity_); // sending stop message
    }
  }

  void shutdown()
  {
    // call shutdown service with empty service to shutdown motors
    std_srvs::Empty empty;
    motor_service_client.call(empty); 
  }

};

} // namespace rosplan_interface_quadrotor


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosplan_interface_quadrotor");
  ros::NodeHandle nh;

  // initialize actionserver name by request to ROS parameter server
	std::string actionserver;
	nh.param("nav_server", actionserver, std::string("/nav_server"));

  rosplan_interface_quadrotor::Quadrotor quad_iface(nh,actionserver);

  ros::Subscriber action_sub = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &rosplan_interface_quadrotor::Quadrotor::dispatchCallback, &quad_iface);

  ros::Subscriber height_sub = nh.subscribe("/sonar_height", 1000, &rosplan_interface_quadrotor::Quadrotor::heightCallback, &quad_iface);
  
  ros::spin();

  return 0;
}
