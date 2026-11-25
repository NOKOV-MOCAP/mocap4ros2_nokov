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

#ifndef MOCAP4R2_NOKOV_DRIVER__NOKOV_FRAME_HPP_
#define MOCAP4R2_NOKOV_DRIVER__NOKOV_FRAME_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "mocap4r2_control/frame_publisher.hpp"
#include "rclcpp/time.hpp"

namespace mocap4r2_nokov_driver
{

enum class UpAxis
{
  Z,
  Y
};

// A marker as measured by the Nokov SDK, in the units of the SDK (millimetres).
struct NokovMarker
{
  int id{-1};                // Stable id from the SDK, -1 when unknown
  std::string name;          // Label from the SDK, empty when unlabeled
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float residual{-1.0F};     // Reconstruction residual, negative when unknown
  bool occluded{false};
};

// A rigid body (or a skeleton bone) as measured by the Nokov SDK.
struct NokovBody
{
  int id{-1};                // Stable id from the SDK, -1 when unknown
  int parent_id{-1};         // Parent body, -1 for a root body
  std::string name;          // Name given in the Nokov software
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float qx{0.0F};
  float qy{0.0F};
  float qz{0.0F};
  float qw{1.0F};
  float mean_error{-1.0F};   // Mean marker error, negative when unknown
  bool tracked{false};       // False when the body was not solved in this frame
  std::vector<NokovMarker> markers;
};

// An articulated body as measured by the Nokov SDK.
struct NokovSkeleton
{
  int id{-1};
  std::string name;
  bool tracked{false};
  std::vector<NokovBody> bones;
};

// One frame of Nokov data, copied out of the buffers owned by the SDK.
struct NokovFrame
{
  int64_t frame_number{0};
  double latency{0.0};       // Seconds between the capture and the transmission
  std::vector<NokovMarker> markers;         // Unlabeled markers
  std::vector<NokovBody> rigid_bodies;
  std::vector<NokovSkeleton> skeletons;
};

struct NokovBoneDescription
{
  std::string name;
  int parent_id{-1};
};

mocap4r2_control::MocapFrame to_mocap_frame(
  const NokovFrame & frame,
  const rclcpp::Time & capture_time,
  UpAxis up_axis);

}  // namespace mocap4r2_nokov_driver

#endif  // MOCAP4R2_NOKOV_DRIVER__NOKOV_FRAME_HPP_
