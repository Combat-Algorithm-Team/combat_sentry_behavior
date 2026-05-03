// Copyright COMBAT
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

#ifndef COMBAT_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_SENTRY_STATUS_HPP_
#define COMBAT_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_SENTRY_STATUS_HPP_

#include "combat_sentry_behavior/plugins/action/pub_uint8_base.hpp"

namespace combat_sentry_behavior
{
class PublishSentryStatusAction : public PubUInt8Base
{
public:
  PublishSentryStatusAction(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();
};

}  // namespace combat_sentry_behavior

#endif  // COMBAT_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_SENTRY_STATUS_HPP_