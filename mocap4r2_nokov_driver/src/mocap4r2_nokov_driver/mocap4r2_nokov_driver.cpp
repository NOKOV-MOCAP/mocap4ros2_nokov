// Copyright 2021 Institute for Robotics and Intelligent Machines,
//                Georgia Institute of Technology
// Copyright 2024 Intelligent Robotics Lab
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
//
// Author: Christian Llanes <christian.llanes@gatech.edu>
// Author: David Vargas Frutos <david.vargas@urjc.es>
// Author: Francisco Martín <fmrico@urjc.es>

#include <string>
#include <vector>
#include <memory>

#include "mocap4r2_msgs/msg/marker.hpp"
#include "mocap4r2_msgs/msg/markers.hpp"

#include "mocap4r2_nokov_driver/mocap4r2_nokov_driver.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace mocap4r2_nokov_driver
{

using std::placeholders::_1;
using std::placeholders::_2;
std::mutex mtx;
NokovDriverNode::NokovDriverNode()
: ControlledLifecycleNode("mocap4r2_nokov_driver_node")
{
  declare_parameter<std::string>("server_address", "10.1.1.198");
  client.reset(new NokovSDKClient());
  client->SetDataCallback(process_frame_callback, this);
}

NokovDriverNode::~NokovDriverNode()
{
}

bool NokovDriverNode::stop_nokov()
{
  RCLCPP_INFO(get_logger(), "Disconnecting from nokov DataStream SDK");

  return true;
}

void
NokovDriverNode::control_start(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
}

void
NokovDriverNode::control_stop(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
}

void XINGYING_CALLCONV process_frame_callback(sFrameOfMocapData * data, void * pUserData)
{
  static_cast<NokovDriverNode *>(pUserData)->process_frame(data);
}

void NokovDriverNode::process_frame(sFrameOfMocapData * data)
{
  if (nullptr == data) {
    return;
  }
  std::lock_guard<std::mutex> lck(mtx);
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }
  std::map<int, std::vector<mocap4r2_msgs::msg::Marker>> marker2rb;
  // Markers
  if (mocap4r2_markers_pub_->get_subscription_count() > 0) {
    mocap4r2_msgs::msg::Markers msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.frame_number = data->iFrame;

    for (int i = 0; i < data->nOtherMarkers; i++) {
      mocap4r2_msgs::msg::Marker marker;
      marker.id_type = mocap4r2_msgs::msg::Marker::USE_INDEX;
      marker.marker_index = i;
      marker.translation.x = data->OtherMarkers[i][0] * 0.001f;
      marker.translation.y = data->OtherMarkers[i][1] * 0.001f;
      marker.translation.z = data->OtherMarkers[i][2] * 0.001f;

      msg.markers.push_back(marker);
    }
    mocap4r2_markers_pub_->publish(msg);
  }

  if (mocap4r2_rigid_body_pub_->get_subscription_count() > 0) {
    mocap4r2_msgs::msg::RigidBodies msg_rb;
    msg_rb.header.stamp = now();
    msg_rb.header.frame_id = "map";
    msg_rb.frame_number = data->iFrame;

    for (int i = 0; i < data->nRigidBodies; i++) {
      mocap4r2_msgs::msg::RigidBody rb;

      rb.rigid_body_name = data->MocapData[i].szName;
      rb.pose.position.x = data->RigidBodies[i].x * 0.001f;
      rb.pose.position.y = data->RigidBodies[i].y * 0.001f;
      rb.pose.position.z = data->RigidBodies[i].z * 0.001f;
      rb.pose.orientation.x = data->RigidBodies[i].qx;
      rb.pose.orientation.y = data->RigidBodies[i].qy;
      rb.pose.orientation.z = data->RigidBodies[i].qz;
      rb.pose.orientation.w = data->RigidBodies[i].qw;

      mocap4r2_msgs::msg::Markers msg;
      msg.header.stamp = msg_rb.header.stamp;
      msg.header.frame_id = "map";
      msg.frame_number = msg_rb.frame_number;
      for (int j = 0; j < data->RigidBodies[i].nMarkers; j++) {
        mocap4r2_msgs::msg::Marker marker;
        marker.id_type = mocap4r2_msgs::msg::Marker::USE_INDEX;
        marker.marker_index = j;
        marker.translation.x = data->RigidBodies[i].Markers[j][0] * 0.001f;
        marker.translation.y = data->RigidBodies[i].Markers[j][1] * 0.001f;
        marker.translation.z = data->RigidBodies[i].Markers[j][2] * 0.001f;
        msg.markers.push_back(marker);
      }
      rb.markers = msg.markers;
      msg_rb.rigidbodies.push_back(rb);
    }
    mocap4r2_rigid_body_pub_->publish(msg_rb);
  }
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// The next Callbacks are used to manage behavior in the different states of the lifecycle node.
CallbackReturnT
NokovDriverNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  initParameters();

  mocap4r2_markers_pub_ = create_publisher<mocap4r2_msgs::msg::Markers>(
    "markers", rclcpp::QoS(1000));
  mocap4r2_rigid_body_pub_ = create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "rigid_bodies", rclcpp::QoS(1000));
  connect_nokov();

  RCLCPP_INFO(get_logger(), "Configured!\n");
  return ControlledLifecycleNode::on_configure(state);
}

CallbackReturnT
NokovDriverNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  mocap4r2_markers_pub_->on_activate();
  mocap4r2_rigid_body_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "Activated!\n");

  return ControlledLifecycleNode::on_activate(state);
}

CallbackReturnT
NokovDriverNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  mocap4r2_markers_pub_->on_deactivate();
  mocap4r2_rigid_body_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated!\n");

  return ControlledLifecycleNode::on_deactivate(state);
}

CallbackReturnT
NokovDriverNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaned up!\n");

  if (disconnect_nokov()) {
    return ControlledLifecycleNode::on_cleanup(state);
  } else {
    return CallbackReturnT::FAILURE;
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
NokovDriverNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutted down!\n");

  if (disconnect_nokov()) {
    return ControlledLifecycleNode::on_shutdown(state);
  } else {
    return CallbackReturnT::FAILURE;
  }
}

CallbackReturnT
NokovDriverNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  disconnect_nokov();

  return ControlledLifecycleNode::on_error(state);
}

bool
NokovDriverNode::connect_nokov()
{
  RCLCPP_INFO(get_logger(), "Trying to connect to Nokov SDK at %s ...", server_address_.c_str());

  client->Uninitialize();
  while (rclcpp::ok() && client->Initialize(const_cast<char *>(server_address_.c_str()))) {
    RCLCPP_WARN(get_logger(), "Connecting to server again");
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  RCLCPP_WARN(get_logger(), "Connecting SUCCESSFUL");

  return true;
}

bool
NokovDriverNode::disconnect_nokov()
{
  client->Uninitialize();
  return true;
}

void
NokovDriverNode::initParameters()
{
  get_parameter<std::string>("server_address", server_address_);
}

}  // namespace mocap4r2_nokov_driver
