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

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "mocap4r2_interfaces/msg/marker.hpp"
#include "mocap4r2_interfaces/msg/rigid_body.hpp"
#include "mocap4r2_interfaces/msg/segment.hpp"
#include "mocap4r2_interfaces/msg/skeleton.hpp"

#include "mocap4r2_nokov_driver/nokov_frame.hpp"

namespace mocap4r2_nokov_driver
{

namespace
{

constexpr double kMillimetresToMetres = 0.001;

// A rotation matrix of the body frame, row major.
using Matrix3 = double[9];

void
quaternion_to_matrix(double qx, double qy, double qz, double qw, Matrix3 out)
{
  out[0] = 1.0 - 2.0 * (qy * qy + qz * qz);
  out[1] = 2.0 * (qx * qy - qz * qw);
  out[2] = 2.0 * (qx * qz + qy * qw);
  out[3] = 2.0 * (qx * qy + qz * qw);
  out[4] = 1.0 - 2.0 * (qx * qx + qz * qz);
  out[5] = 2.0 * (qy * qz - qx * qw);
  out[6] = 2.0 * (qx * qz - qy * qw);
  out[7] = 2.0 * (qy * qz + qx * qw);
  out[8] = 1.0 - 2.0 * (qx * qx + qy * qy);
}

void
matrix_to_quaternion(const Matrix3 & m, double & qx, double & qy, double & qz, double & qw)
{
  const double trace = m[0] + m[4] + m[8];
  if (trace > 0.0) {
    const double s = 0.5 / std::sqrt(trace + 1.0);
    qw = 0.25 / s;
    qx = (m[7] - m[5]) * s;
    qy = (m[2] - m[6]) * s;
    qz = (m[3] - m[1]) * s;
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const double s = 2.0 * std::sqrt(1.0 + m[0] - m[4] - m[8]);
    qw = (m[7] - m[5]) / s;
    qx = 0.25 * s;
    qy = (m[1] + m[3]) / s;
    qz = (m[2] + m[6]) / s;
  } else if (m[4] > m[8]) {
    const double s = 2.0 * std::sqrt(1.0 + m[4] - m[0] - m[8]);
    qw = (m[2] - m[6]) / s;
    qx = (m[1] + m[3]) / s;
    qy = 0.25 * s;
    qz = (m[5] + m[7]) / s;
  } else {
    const double s = 2.0 * std::sqrt(1.0 + m[8] - m[0] - m[4]);
    qw = (m[3] - m[1]) / s;
    qx = (m[2] + m[6]) / s;
    qy = (m[5] + m[7]) / s;
    qz = 0.25 * s;
  }
}

const Matrix3 kYupToZup = {1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0};

void
multiply(const Matrix3 & a, const Matrix3 & b, Matrix3 out)
{
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out[i * 3 + j] = a[i * 3] * b[j] +
        a[i * 3 + 1] * b[3 + j] +
        a[i * 3 + 2] * b[6 + j];
    }
  }
}

void
transpose(const Matrix3 & a, Matrix3 out)
{
  out[0] = a[0];  out[1] = a[3];  out[2] = a[6];
  out[3] = a[1];  out[4] = a[4];  out[5] = a[7];
  out[6] = a[2];  out[7] = a[5];  out[8] = a[8];
}

// Position of a point of the Nokov frame, in metres and in the published axes.
void
convert_position(float x, float y, float z, UpAxis up_axis, double & ox, double & oy, double & oz)
{
  const double mx = x * kMillimetresToMetres;
  const double my = y * kMillimetresToMetres;
  const double mz = z * kMillimetresToMetres;
  if (up_axis == UpAxis::Y) {
    ox = mx;
    oy = -mz;
    oz = my;
  } else {
    ox = mx;
    oy = my;
    oz = mz;
  }
}

// Orientation of a body of the Nokov frame, in the published axes.
void
convert_orientation(
  double qx, double qy, double qz, double qw, UpAxis up_axis,
  double & ox, double & oy, double & oz, double & ow)
{
  if (up_axis == UpAxis::Z) {
    ox = qx;
    oy = qy;
    oz = qz;
    ow = qw;
    return;
  }
  Matrix3 m;
  quaternion_to_matrix(qx, qy, qz, qw, m);
  // result = P * m * P^T
  Matrix3 p_transposed;
  transpose(kYupToZup, p_transposed);
  Matrix3 tmp;
  Matrix3 result;
  multiply(kYupToZup, m, tmp);
  multiply(tmp, p_transposed, result);
  matrix_to_quaternion(result, ox, oy, oz, ow);
}

mocap4r2_interfaces::msg::Marker
convert_marker(const NokovMarker & marker, UpAxis up_axis, bool occluded)
{
  mocap4r2_interfaces::msg::Marker out;
  out.id = marker.id;
  out.name = marker.name;
  out.occluded = occluded || marker.occluded;
  out.residual = marker.residual < 0.0F ? marker.residual :
    static_cast<float>(marker.residual * kMillimetresToMetres);
  convert_position(
    marker.x, marker.y, marker.z, up_axis,
    out.position.x, out.position.y, out.position.z);
  return out;
}

void
convert_body(
  const NokovBody & body, UpAxis up_axis,
  mocap4r2_interfaces::msg::RigidBody & out)
{
  out.id = body.id;
  out.name = body.name;
  out.tracking_valid = body.tracked;
  out.mean_error = body.mean_error < 0.0F ? body.mean_error :
    static_cast<float>(body.mean_error * kMillimetresToMetres);
  convert_position(
    body.x, body.y, body.z, up_axis,
    out.pose.position.x, out.pose.position.y, out.pose.position.z);
  convert_orientation(
    body.qx, body.qy, body.qz, body.qw, up_axis,
    out.pose.orientation.x, out.pose.orientation.y,
    out.pose.orientation.z, out.pose.orientation.w);
  out.markers.reserve(body.markers.size());
  for (const auto & marker : body.markers) {
    out.markers.push_back(convert_marker(marker, up_axis, !body.tracked));
  }
}

}  // namespace

namespace
{

// The global number of a bone in the "eBONE" enumeration of the system
int ebone_id_from_name(const std::string & name)
{
  static const std::map<std::string, int> ebone_ids = {
    {"pelvis", 2},
    {"spine", 3},
    {"spine1", 4},
    {"spine2", 5},
    {"spine3", 58},
    {"spine4", 59},
    {"neck", 44},
    {"neck1", 45},
    {"head", 46},
    {"headend", 47},
    {"head_end", 47},
    {"leftshoulder", 6},
    {"l.shoulder", 6},
    {"l.upperarm", 7},
    {"l.forearm", 8},
    {"l.hand", 9},
    {"l.thumb1", 10},
    {"l.thumb2", 11},
    {"l.thumb3", 12},
    {"l.index1", 13},
    {"l.index2", 14},
    {"l.index3", 15},
    {"l.middle1", 16},
    {"l.middle2", 17},
    {"l.middle3", 18},
    {"l.ring1", 19},
    {"l.ring2", 20},
    {"l.ring3", 21},
    {"l.pinky1", 22},
    {"l.pinky2", 23},
    {"l.pinky3", 24},
    {"rightshoulder", 25},
    {"r.shoulder", 25},
    {"r.upperarm", 26},
    {"r.forearm", 27},
    {"r.hand", 28},
    {"r.thumb1", 29},
    {"r.thumb2", 30},
    {"r.thumb3", 31},
    {"r.index1", 32},
    {"r.index2", 33},
    {"r.index3", 34},
    {"r.middle1", 35},
    {"r.middle2", 36},
    {"r.middle3", 37},
    {"r.ring1", 38},
    {"r.ring2", 39},
    {"r.ring3", 40},
    {"r.pinky1", 41},
    {"r.pinky2", 42},
    {"r.pinky3", 43},
    {"leftupleg", 48},
    {"l.thigh", 48},
    {"leftleg", 49},
    {"l.shank", 49},
    {"leftfoot", 50},
    {"l.foot", 50},
    {"lefttoebase", 51},
    {"l.ball", 51},
    {"l.toe", 51},
    {"rightupleg", 53},
    {"r.thigh", 53},
    {"rightleg", 54},
    {"r.shank", 54},
    {"rightfoot", 55},
    {"r.foot", 55},
    {"righttoebase", 56},
    {"r.ball", 56},
    {"r.toe", 56},
    {"trunk", 3},
    {"head/neck", 46},
    {"pelviswrtlab", 1},
    {"trunkwrtlab", 1},
  };

  std::string lower;
  lower.reserve(name.size());
  for (const char c : name) {
    lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  const auto found = ebone_ids.find(lower);
  return found != ebone_ids.end() ? found->second : 0;
}

}  // namespace

std::vector<NokovBoneDescription>
resolve_bones(
  const std::vector<int> & parent_ids,
  const std::vector<std::string> & names)
{
  std::map<int, int> id_to_index;
  for (std::size_t i = 0; i < names.size(); ++i) {
    const int id = ebone_id_from_name(names[i]);
    if (id != 0) {
      id_to_index[id] = static_cast<int>(i);
    }
  }

  std::vector<NokovBoneDescription> bones;
  bones.reserve(names.size());
  for (std::size_t i = 0; i < names.size(); ++i) {
    NokovBoneDescription bone;
    bone.name = names[i];
    // The model definition refers to the parent of a bone by its global
    // number; translate it into the local numbering.
    const int parent_id = i < parent_ids.size() ? parent_ids[i] : 0;
    const auto parent = id_to_index.find(parent_id);
    if (parent != id_to_index.end() && parent->second != static_cast<int>(i)) {
      bone.parent_segment = parent->second + 1;
    }
    bones.push_back(std::move(bone));
  }
  return bones;
}

mocap4r2_control::MocapFrame
to_mocap_frame(
  const NokovFrame & frame,
  const rclcpp::Time & capture_time,
  UpAxis up_axis)
{
  mocap4r2_control::MocapFrame out;
  out.frame_number = static_cast<uint64_t>(std::max<int64_t>(frame.frame_number, 0));
  out.capture_time = capture_time;

  out.markers.reserve(frame.markers.size());
  for (const auto & marker : frame.markers) {
    out.markers.push_back(convert_marker(marker, up_axis, false));
  }

  out.rigid_bodies.reserve(frame.rigid_bodies.size());
  for (const auto & body : frame.rigid_bodies) {
    mocap4r2_interfaces::msg::RigidBody rigid_body;
    convert_body(body, up_axis, rigid_body);
    out.rigid_bodies.push_back(std::move(rigid_body));
  }

  out.skeletons.reserve(frame.skeletons.size());
  for (const auto & skeleton : frame.skeletons) {
    mocap4r2_interfaces::msg::Skeleton sk;
    sk.id = skeleton.id;
    sk.name = skeleton.name;
    sk.tracking_valid = skeleton.tracked;
    sk.segments.reserve(skeleton.bones.size());
    for (const auto & bone : skeleton.bones) {
      mocap4r2_interfaces::msg::Segment segment;
      segment.id = bone.id;
      segment.name = bone.name;
      segment.parent_id = bone.parent_id;
      segment.tracking_valid = bone.tracked;
      convert_position(
        bone.x, bone.y, bone.z, up_axis,
        segment.pose.position.x, segment.pose.position.y, segment.pose.position.z);
      convert_orientation(
        bone.qx, bone.qy, bone.qz, bone.qw, up_axis,
        segment.pose.orientation.x, segment.pose.orientation.y,
        segment.pose.orientation.z, segment.pose.orientation.w);
      sk.segments.push_back(std::move(segment));
    }
    out.skeletons.push_back(std::move(sk));
  }

  return out;
}

}  // namespace mocap4r2_nokov_driver
