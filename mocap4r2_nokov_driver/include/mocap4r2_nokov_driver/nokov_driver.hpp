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

#ifndef MOCAP4R2_NOKOV_DRIVER__NOKOV_DRIVER_HPP_
#define MOCAP4R2_NOKOV_DRIVER__NOKOV_DRIVER_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "mocap4r2_control/frame_publisher.hpp"
#include "mocap4r2_control/mocap_system_node.hpp"
#include "mocap4r2_nokov_driver/nokov_frame.hpp"

#include <NokovSDKClient.h>
#include <NokovSDKTypes.h>

namespace mocap4r2_nokov_driver
{

// Driver of a Nokov motion capture system.
class NokovDriver : public mocap4r2_control::MocapSystemNode
{
public:
  explicit NokovDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~NokovDriver() override;

  // Callback of the SDK: it runs in the thread of the SDK.
  void process_frame(sFrameOfMocapData * data);

protected:
  CallbackReturn on_system_configure() override;
  CallbackReturn on_system_activate() override;
  CallbackReturn on_system_deactivate() override;
  CallbackReturn on_system_cleanup() override;

  double nominal_frame_rate() const override {return nominal_frame_rate_;}
  std::string server() const override;
  bool connected() const override;

private:
  void connect_nokov();
  void disconnect_nokov();

  // Names of the bodies, as defined in the Nokov software.
  void refresh_descriptions();

  // Stop the SDK thread before the node goes away.
  void stop();

  std::unique_ptr<NokovSDKClient> client_;
  std::unique_ptr<mocap4r2_control::FramePublisher> publisher_;

  std::string server_address_;
  double nominal_frame_rate_{0.0};
  double timeout_{10.0};
  UpAxis up_axis_{UpAxis::Z};

  std::mutex descriptions_mutex_;
  std::map<int, std::string> rigid_body_names_;
  std::map<int, std::string> skeleton_names_;
  std::map<int, std::vector<NokovBoneDescription>> skeleton_bones_;
  std::map<std::string, std::vector<std::string>> marker_set_names_;

  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
};

}  // namespace mocap4r2_nokov_driver

#endif  // MOCAP4R2_NOKOV_DRIVER__NOKOV_DRIVER_HPP_
