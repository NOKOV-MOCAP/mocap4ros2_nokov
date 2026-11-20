// Copyright 2026 Intelligent Robotics Lab
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
#include <cstring>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "mocap4r2_control/frame_publisher.hpp"
#include "mocap4r2_control/mocap_system_node.hpp"

#include "mocap4r2_nokov_driver/nokov_driver.hpp"

#include <NokovSDKClient.h>
#include <NokovSDKTypes.h>

namespace mocap4r2_nokov_driver
{

namespace
{

// Callback of the SDK. It is called from the thread of the SDK, where the
// frame is only valid during the call.
void XINGYING_CALLCONV process_frame_callback(sFrameOfMocapData * data, void * pUserData)
{
  if (pUserData != nullptr && data != nullptr) {
    static_cast<NokovDriver *>(pUserData)->process_frame(data);
  }
}

// The SDK measures the delay between the capture and the transmission in
// milliseconds.
constexpr double kMillisecondsToSeconds = 0.001;

}  // namespace

NokovDriver::NokovDriver(const rclcpp::NodeOptions & options)
: mocap4r2_control::MocapSystemNode("nokov_driver", "nokov", options)
{
  declare_parameter<std::string>("server_address", "192.168.50.177");
  declare_parameter<double>("nominal_frame_rate", 0.0);
  declare_parameter<double>("timeout", 10.0);
  declare_parameter<std::string>("up_axis", "z");
  mocap4r2_control::declare_frame_publisher_parameters(*this);

  client_ = std::make_unique<NokovSDKClient>();
}

NokovDriver::~NokovDriver()
{
  stop();
}

NokovDriver::CallbackReturn
NokovDriver::on_system_configure()
{
  server_address_ = get_parameter("server_address").as_string();
  nominal_frame_rate_ = get_parameter("nominal_frame_rate").as_double();
  timeout_ = get_parameter("timeout").as_double();

  const auto up_axis = get_parameter("up_axis").as_string();
  if (up_axis == "z" || up_axis == "Z") {
    up_axis_ = UpAxis::Z;
  } else if (up_axis == "y" || up_axis == "Y") {
    up_axis_ = UpAxis::Y;
  } else {
    RCLCPP_ERROR(get_logger(), "up_axis must be 'z' or 'y' (got '%s')", up_axis.c_str());
    return CallbackReturn::FAILURE;
  }

  connect_nokov();
  publisher_ = std::make_unique<mocap4r2_control::FramePublisher>(*this);
  return CallbackReturn::SUCCESS;
}

NokovDriver::CallbackReturn
NokovDriver::on_system_activate()
{
  if (!connected_) {
    connect_nokov();
  }
  client_->SetDataCallback(process_frame_callback, this);
  running_ = true;
  return CallbackReturn::SUCCESS;
}

NokovDriver::CallbackReturn
NokovDriver::on_system_deactivate()
{
  stop();
  return CallbackReturn::SUCCESS;
}

NokovDriver::CallbackReturn
NokovDriver::on_system_cleanup()
{
  stop();
  publisher_.reset();
  return CallbackReturn::SUCCESS;
}

std::string
NokovDriver::server() const
{
  return "Nokov SDK at " + server_address_;
}

bool
NokovDriver::connected() const
{
  return connected_;
}

void
NokovDriver::connect_nokov()
{
  running_ = false;
  RCLCPP_INFO(
    get_logger(), "Connecting to the Nokov SDK at %s ...", server_address_.c_str());

  client_->Uninitialize();
  connected_ = false;

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(timeout_));

  while (true) {
    if (client_->Initialize(const_cast<char *>(server_address_.c_str())) == 0) {
      connected_ = true;
      break;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error(
              "cannot connect to the Nokov SDK at " + server_address_);
    }
    RCLCPP_WARN(
      get_logger(), "Cannot connect to the Nokov SDK at %s, retrying",
      server_address_.c_str());
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  refresh_descriptions();
  // RCLCPP_INFO(get_logger(), "Connected to the Nokov SDK at %s", server_address_.c_str());
}

void
NokovDriver::disconnect_nokov()
{
  if (client_) {
    client_->Uninitialize();
  }
  connected_ = false;
}

void
NokovDriver::refresh_descriptions()
{
  std::map<int, std::string> rigid_body_names;
  std::map<int, std::string> skeleton_names;
  std::map<int, std::vector<NokovBoneDescription>> skeleton_bones;
  std::map<std::string, std::vector<std::string>> marker_set_names;

  sDataDescriptions * descriptions = nullptr;
  if (client_->GetDataDescriptions(&descriptions) != 0 || descriptions == nullptr) {
    RCLCPP_WARN(get_logger(), "The Nokov SDK did not report the model definition");
    return;
  }

  for (int i = 0; i < descriptions->nDataDescriptions; ++i) {
    const auto & description = descriptions->arrDataDescriptions[i];
    switch (description.type) {
      case Descriptor_RigidBody: {
          const auto * body = description.Data.RigidBodyDescription;
          if (body != nullptr) {
            rigid_body_names[body->ID] = std::string(body->szName);
          }
          break;
        }
      case Descriptor_Skeleton: {
          const auto * skeleton = description.Data.SkeletonDescription;
          if (skeleton != nullptr) {
            skeleton_names[skeleton->skeletonID] = std::string(skeleton->szName);
            std::vector<int> parent_ids;
            std::vector<std::string> names;
            parent_ids.reserve(skeleton->nRigidBodies);
            names.reserve(skeleton->nRigidBodies);
            for (int j = 0; j < skeleton->nRigidBodies; ++j) {
              const auto & bone = skeleton->RigidBodies[j];
              parent_ids.push_back(bone.parentID);
              names.emplace_back(bone.szName);
            }
            const auto bones = resolve_bones(parent_ids, names);
            skeleton_bones[skeleton->skeletonID] = std::move(bones);
          }
          break;
        }
      case Descriptor_MarkerSet:
      case Descriptor_MarkerSetEx: {
          const auto * set = description.Data.MarkerSetDescription;
          if (set != nullptr) {
            std::vector<std::string> names;
            names.reserve(set->nMarkers);
            for (int j = 0; j < set->nMarkers; ++j) {
              names.emplace_back(set->szMarkerNames != nullptr ? set->szMarkerNames[j] : "");
            }
            marker_set_names[std::string(set->szName)] = std::move(names);
          }
          break;
        }
      default:
        break;
    }
  }
  client_->FreeDataDescriptions(descriptions);

  std::lock_guard<std::mutex> lock(descriptions_mutex_);
  rigid_body_names_ = std::move(rigid_body_names);
  skeleton_names_ = std::move(skeleton_names);
  skeleton_bones_ = std::move(skeleton_bones);
  marker_set_names_ = std::move(marker_set_names);
}

void
NokovDriver::stop()
{
  running_ = false;
  disconnect_nokov();
}

void
NokovDriver::process_frame(sFrameOfMocapData * data)
{
  if (data == nullptr || !running_ || !publisher_) {
    return;
  }

  std::map<int, std::string> rigid_body_names;
  std::map<int, std::string> skeleton_names;
  std::map<int, std::vector<NokovBoneDescription>> skeleton_bones;
  std::map<std::string, std::vector<std::string>> marker_set_names;
  {
    std::lock_guard<std::mutex> lock(descriptions_mutex_);
    rigid_body_names = rigid_body_names_;
    skeleton_names = skeleton_names_;
    skeleton_bones = skeleton_bones_;
    marker_set_names = marker_set_names_;
  }

  NokovFrame frame;
  frame.frame_number = data->iFrame;
  frame.latency = data->fLatency * kMillisecondsToSeconds;

  // Markers of the marker sets: the SDK gives their position but not their id,
  // so they are identified by the name of the model definition.
  for (int i = 0; i < data->nMarkerSets; ++i) {
    const auto & set = data->MocapData[i];
    const auto found = marker_set_names.find(std::string(set.szName));
    const std::vector<std::string> names =
      found != marker_set_names.end() ? found->second : std::vector<std::string>();
    for (int j = 0; j < set.nMarkers; ++j) {
      NokovMarker marker;
      marker.name = j < static_cast<int>(names.size()) ? names[j] : "";
      marker.x = set.Markers[j][0];
      marker.y = set.Markers[j][1];
      marker.z = set.Markers[j][2];
      frame.markers.push_back(std::move(marker));
    }
  }

  // Labeled markers that do not belong to a marker set.
  for (int i = 0; i < data->nLabeledMarkers; ++i) {
    const auto & marker = data->LabeledMarkers[i];
    NokovMarker labeled;
    labeled.id = marker.ID;
    labeled.x = marker.x;
    labeled.y = marker.y;
    labeled.z = marker.z;
    frame.markers.push_back(std::move(labeled));
  }

  // Unlabeled markers: the SDK does not identify them.
  for (int i = 0; i < data->nOtherMarkers; ++i) {
    NokovMarker unlabeled;
    unlabeled.x = data->OtherMarkers[i][0];
    unlabeled.y = data->OtherMarkers[i][1];
    unlabeled.z = data->OtherMarkers[i][2];
    frame.markers.push_back(std::move(unlabeled));
  }

  // Rigid bodies.
  for (int i = 0; i < data->nRigidBodies; ++i) {
    const auto & body = data->RigidBodies[i];
    NokovBody rigid_body;
    rigid_body.id = body.ID;
    const auto found = rigid_body_names.find(body.ID);
    if (found != rigid_body_names.end()) {
      rigid_body.name = found->second;
    }
    rigid_body.x = body.x;
    rigid_body.y = body.y;
    rigid_body.z = body.z;
    rigid_body.qx = body.qx;
    rigid_body.qy = body.qy;
    rigid_body.qz = body.qz;
    rigid_body.qw = body.qw;
    rigid_body.mean_error = body.MeanError;
    rigid_body.tracked = body.MeanError >= 0.0F && body.nMarkers > 0;
    for (int j = 0; j < body.nMarkers; ++j) {
      NokovMarker marker;
      marker.id = body.MarkerIDs != nullptr ? body.MarkerIDs[j] : -1;
      marker.x = body.Markers[j][0];
      marker.y = body.Markers[j][1];
      marker.z = body.Markers[j][2];
      rigid_body.markers.push_back(std::move(marker));
    }
    frame.rigid_bodies.push_back(std::move(rigid_body));
  }

  // Skeletons.
  for (int i = 0; i < data->nSkeletons; ++i) {
    const auto & skeleton = data->Skeletons[i];
    NokovSkeleton articulated;
    articulated.id = skeleton.skeletonID;
    const auto found = skeleton_names.find(skeleton.skeletonID);
    if (found != skeleton_names.end()) {
      articulated.name = found->second;
    }
    const auto bones_found = skeleton_bones.find(skeleton.skeletonID);
    const std::vector<NokovBoneDescription> bones =
      bones_found != skeleton_bones.end() ? bones_found->second :
      std::vector<NokovBoneDescription>();
    for (int j = 0; j < skeleton.nRigidBodies; ++j) {
      const auto & bone = skeleton.RigidBodyData[j];
      NokovBody segment;
      segment.id = j + 1;
      if (j < static_cast<int>(bones.size())) {
        segment.name = bones[j].name;
        segment.parent_id = bones[j].parent_segment;
      }
      if (segment.name.empty()) {
        segment.name = std::to_string(segment.id);
      }
      segment.x = bone.x;
      segment.y = bone.y;
      segment.z = bone.z;
      segment.qx = bone.qx;
      segment.qy = bone.qy;
      segment.qz = bone.qz;
      segment.qw = bone.qw;
      segment.mean_error = bone.MeanError;
      segment.tracked = bone.MeanError >= 0.0F && bone.nMarkers > 0;
      for (int k = 0; k < bone.nMarkers; ++k) {
        NokovMarker marker;
        marker.id = bone.MarkerIDs != nullptr ? bone.MarkerIDs[k] : -1;
        marker.x = bone.Markers[k][0];
        marker.y = bone.Markers[k][1];
        marker.z = bone.Markers[k][2];
        segment.markers.push_back(std::move(marker));
      }
      articulated.bones.push_back(std::move(segment));
    }
    articulated.tracked = false;
    for (const auto & bone : articulated.bones) {
      articulated.tracked = articulated.tracked || bone.tracked;
    }
    frame.skeletons.push_back(std::move(articulated));
  }

  const auto capture_time = now() - rclcpp::Duration::from_seconds(frame.latency);
  publisher_->publish(to_mocap_frame(frame, capture_time, up_axis_));
}

}  // namespace mocap4r2_nokov_driver
