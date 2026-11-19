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

#include <gtest/gtest.h>

#include <cmath>

#include "mocap4r2_nokov_driver/nokov_frame.hpp"

using mocap4r2_nokov_driver::NokovBody;
using mocap4r2_nokov_driver::NokovBoneDescription;
using mocap4r2_nokov_driver::NokovFrame;
using mocap4r2_nokov_driver::NokovMarker;
using mocap4r2_nokov_driver::NokovSkeleton;
using mocap4r2_nokov_driver::UpAxis;
using mocap4r2_nokov_driver::resolve_bones;
using mocap4r2_nokov_driver::to_mocap_frame;

namespace
{

constexpr double kTolerance = 1e-6;

rclcpp::Time
capture_time()
{
  return rclcpp::Time(1234, 0, RCL_ROS_TIME);
}

}  // namespace

// The SDK measures in millimetres, the driver publishes metres.
TEST(NokovFrame, ConvertsMillimetresToMetres)
{
  NokovFrame frame;
  frame.frame_number = 7;
  NokovMarker marker;
  marker.x = 1000.0F;
  marker.y = -2000.0F;
  marker.z = 3000.0F;
  frame.markers.push_back(marker);

  const auto out = to_mocap_frame(frame, capture_time(), UpAxis::Z);

  ASSERT_EQ(out.markers.size(), 1u);
  EXPECT_NEAR(out.markers[0].position.x, 1.0, kTolerance);
  EXPECT_NEAR(out.markers[0].position.y, -2.0, kTolerance);
  EXPECT_NEAR(out.markers[0].position.z, 3.0, kTolerance);
}

// The Nokov software is set up with Z-up by default, so the axes are kept.
TEST(NokovFrame, KeepsTheAxesOfAZUpSystem)
{
  NokovFrame frame;
  NokovBody body;
  body.id = 3;
  body.x = 10.0F;
  body.y = 20.0F;
  body.z = 30.0F;
  body.tracked = true;
  frame.rigid_bodies.push_back(body);

  const auto out = to_mocap_frame(frame, capture_time(), UpAxis::Z);

  ASSERT_EQ(out.rigid_bodies.size(), 1u);
  EXPECT_NEAR(out.rigid_bodies[0].pose.position.x, 0.01, kTolerance);
  EXPECT_NEAR(out.rigid_bodies[0].pose.position.y, 0.02, kTolerance);
  EXPECT_NEAR(out.rigid_bodies[0].pose.position.z, 0.03, kTolerance);
}

// A Y-up system is rotated into Z-up by the driver.
TEST(NokovFrame, RotatesAYUpSystemIntoZUp)
{
  NokovFrame frame;
  NokovBody body;
  body.id = 1;
  body.x = 1.0F;
  body.y = 2.0F;
  body.z = 3.0F;
  body.tracked = true;
  frame.rigid_bodies.push_back(body);

  const auto out = to_mocap_frame(frame, capture_time(), UpAxis::Y);

  ASSERT_EQ(out.rigid_bodies.size(), 1u);
  EXPECT_NEAR(out.rigid_bodies[0].pose.position.x, 0.001, kTolerance);
  EXPECT_NEAR(out.rigid_bodies[0].pose.position.y, -0.003, kTolerance);
  EXPECT_NEAR(out.rigid_bodies[0].pose.position.z, 0.002, kTolerance);
}

// The rotation of a body is rotated as well, around the new up axis.
TEST(NokovFrame, RotatesTheOrientationOfAYUpSystem)
{
  const float half = std::sqrt(0.5F);

  NokovFrame frame;
  NokovBody body;
  body.id = 1;
  body.qx = 0.0F;
  body.qy = half;   // 90 degrees around the native up axis
  body.qz = 0.0F;
  body.qw = half;
  body.tracked = true;
  frame.rigid_bodies.push_back(body);

  const auto out = to_mocap_frame(frame, capture_time(), UpAxis::Y);

  ASSERT_EQ(out.rigid_bodies.size(), 1u);
  const auto & q = out.rigid_bodies[0].pose.orientation;
  EXPECT_NEAR(q.x, 0.0, kTolerance);
  EXPECT_NEAR(q.y, 0.0, kTolerance);
  EXPECT_NEAR(q.z, half, kTolerance);
  EXPECT_NEAR(q.w, half, kTolerance);
}

// The ids of the SDK are stable, they are published unchanged.
TEST(NokovFrame, KeepsTheStableIdsOfTheSdk)
{
  NokovFrame frame;
  NokovMarker unlabeled;
  unlabeled.id = -1;
  frame.markers.push_back(unlabeled);

  NokovBody body;
  body.id = 42;
  body.tracked = true;
  NokovMarker marker;
  marker.id = 7;
  body.markers.push_back(marker);
  frame.rigid_bodies.push_back(body);

  const auto out = to_mocap_frame(frame, capture_time(), UpAxis::Z);

  ASSERT_EQ(out.markers.size(), 1u);
  EXPECT_EQ(out.markers[0].id, -1);
  ASSERT_EQ(out.rigid_bodies.size(), 1u);
  EXPECT_EQ(out.rigid_bodies[0].id, 42);
  ASSERT_EQ(out.rigid_bodies[0].markers.size(), 1u);
  EXPECT_EQ(out.rigid_bodies[0].markers[0].id, 7);
}

// A body that was not solved is published as invalid, never as a valid pose.
TEST(NokovFrame, MarksUntrackedBodiesAsInvalid)
{
  NokovFrame frame;
  NokovBody body;
  body.id = 5;
  body.x = 1.0F;
  body.y = 2.0F;
  body.z = 3.0F;
  body.tracked = false;
  NokovMarker marker;
  marker.id = 1;
  body.markers.push_back(marker);
  frame.rigid_bodies.push_back(body);

  const auto out = to_mocap_frame(frame, capture_time(), UpAxis::Z);

  ASSERT_EQ(out.rigid_bodies.size(), 1u);
  EXPECT_FALSE(out.rigid_bodies[0].tracking_valid);
  ASSERT_EQ(out.rigid_bodies[0].markers.size(), 1u);
  EXPECT_TRUE(out.rigid_bodies[0].markers[0].occluded);
}

// The mean error is a length, so it is converted to metres as well.
TEST(NokovFrame, ConvertsTheMeanErrorToMetres)
{
  NokovFrame frame;
  NokovBody body;
  body.id = 1;
  body.mean_error = 2.5F;
  body.tracked = true;
  frame.rigid_bodies.push_back(body);

  const auto out = to_mocap_frame(frame, capture_time(), UpAxis::Z);

  ASSERT_EQ(out.rigid_bodies.size(), 1u);
  EXPECT_NEAR(out.rigid_bodies[0].mean_error, 0.0025F, 1e-6F);
}

// The frame number and the capture time come from the SDK.
TEST(NokovFrame, KeepsTheFrameNumberAndTheCaptureTime)
{
  NokovFrame frame;
  frame.frame_number = 1234;

  const auto out = to_mocap_frame(frame, capture_time(), UpAxis::Z);

  EXPECT_EQ(out.frame_number, 1234u);
  EXPECT_EQ(out.capture_time.nanoseconds(), capture_time().nanoseconds());
}

// The segments of a skeleton keep their parent.
TEST(NokovFrame, PublishesTheSegmentsOfASkeleton)
{
  NokovFrame frame;
  NokovSkeleton skeleton;
  skeleton.id = 2;
  skeleton.name = "human";
  NokovBody root;
  root.id = 10;
  root.parent_id = -1;
  root.tracked = true;
  skeleton.bones.push_back(root);
  NokovBody bone;
  bone.id = 11;
  bone.parent_id = 10;
  bone.tracked = false;
  skeleton.bones.push_back(bone);
  skeleton.tracked = true;
  frame.skeletons.push_back(skeleton);

  const auto out = to_mocap_frame(frame, capture_time(), UpAxis::Z);

  ASSERT_EQ(out.skeletons.size(), 1u);
  EXPECT_EQ(out.skeletons[0].id, 2);
  EXPECT_EQ(out.skeletons[0].name, "human");
  EXPECT_TRUE(out.skeletons[0].tracking_valid);
  ASSERT_EQ(out.skeletons[0].segments.size(), 2u);
  EXPECT_EQ(out.skeletons[0].segments[0].parent_id, -1);
  EXPECT_TRUE(out.skeletons[0].segments[0].tracking_valid);
  EXPECT_EQ(out.skeletons[0].segments[1].parent_id, 10);
  EXPECT_FALSE(out.skeletons[0].segments[1].tracking_valid);
}

TEST(NokovFrame, ResolvesParentsThroughTheEBoneEnumeration)
{
  const std::vector<int> parent_ids = {0, 2, 2, 53, 2};
  const std::vector<std::string> names = {"Pelvis", "R.Thigh", "L.Thigh", "R.Shank", "Trunk"};

  const auto bones = resolve_bones(parent_ids, names);

  ASSERT_EQ(bones.size(), 5u);
  EXPECT_EQ(bones[0].name, "Pelvis");
  EXPECT_EQ(bones[0].parent_segment, -1);   // The root has no parent number.
  EXPECT_EQ(bones[1].parent_segment, 1);    // R.Thigh is a child of Pelvis.
  EXPECT_EQ(bones[2].parent_segment, 1);    // L.Thigh is a child of Pelvis.
  EXPECT_EQ(bones[3].parent_segment, 2);    // R.Shank is a child of R.Thigh.
  EXPECT_EQ(bones[4].parent_segment, 1);    // Trunk is a child of Pelvis.
}

TEST(NokovFrame, LocatesParentsByNumberNotByPosition)
{
  const std::vector<int> parent_ids = {0, 2, 53};
  const std::vector<std::string> names = {"PELVIS", "r.thigh", "R.Shank"};

  const auto bones = resolve_bones(parent_ids, names);

  ASSERT_EQ(bones.size(), 3u);
  EXPECT_EQ(bones[0].parent_segment, -1);
  EXPECT_EQ(bones[1].parent_segment, 1);   // Pelvis has number 2, position 1.
  EXPECT_EQ(bones[2].parent_segment, 2);   // R.Thigh has number 53, position 2.
}

// A parent that is no bone of the skeleton (or the bone itself) is
// published as NO_PARENT.
TEST(NokovFrame, RootsBonesWhoseParentIsOutsideTheSkeleton)
{
  const std::vector<int> parent_ids = {99, 53, 0};
  const std::vector<std::string> names = {"Pelvis", "R.Thigh", "L.Thigh"};

  const auto bones = resolve_bones(parent_ids, names);

  ASSERT_EQ(bones.size(), 3u);
  EXPECT_EQ(bones[0].parent_segment, -1);   // No bone with number 99.
  EXPECT_EQ(bones[1].parent_segment, -1);   // R.Thigh names itself: root.
  EXPECT_EQ(bones[2].parent_segment, -1);   // No parent number.
}

// The parents of a whole standard model of the system, as reported by the
// model definition of a real skeleton.
TEST(NokovFrame, ResolvesTheParentsOfAWholeSkeleton)
{
  const std::vector<int> parent_ids = {0, 2, 2, 53, 48, 54, 49, 2, 3, 3, 3, 26, 7, 27, 8};
  const std::vector<std::string> names = {
    "Pelvis", "R.Thigh", "L.Thigh", "R.Shank", "L.Shank", "R.Foot", "L.Foot",
    "Trunk", "Head/Neck", "R.UpperArm", "L.UpperArm", "R.Forearm", "L.Forearm",
    "R.Hand", "L.Hand"};

  const auto bones = resolve_bones(parent_ids, names);

  ASSERT_EQ(bones.size(), 15u);
  const std::vector<int> expected = {-1, 1, 1, 2, 3, 4, 5, 1, 8, 8, 8, 10, 11, 12, 13};
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(bones[i].parent_segment, expected[i]) << "bone " << i + 1;
  }
}
