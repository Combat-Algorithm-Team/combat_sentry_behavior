/* Copyright (C) 2015-2018 Michele Colledanchise -  All Rights Reserved
 * Copyright (C) 2018-2025 Davide Faconti, Eurecat -  All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include "behaviortree_cpp/control_node.h"

#include <cstddef>
#include <string>

namespace combat_sentry_behavior
{
/**
 * @brief Ticks all children on every tick and evaluates parallel success/failure thresholds.
 *
 * Unlike BT.CPP's built-in Parallel node, this control node does not remember completed
 * children between ticks. A child that returns SUCCESS or FAILURE can be ticked again on the
 * next iteration, matching the "reactive" behavior needed by long-running decision branches.
 */
class ReactiveParallelNode : public BT::ControlNode
{
public:
  ReactiveParallelNode(const std::string & name);

  ReactiveParallelNode(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>(
        THRESHOLD_SUCCESS, -1, "number of children that need to succeed to trigger SUCCESS"),
      BT::InputPort<int>(
        THRESHOLD_FAILURE, 1, "number of children that need to fail to trigger FAILURE")};
  }

  ~ReactiveParallelNode() override = default;

  ReactiveParallelNode(const ReactiveParallelNode &) = delete;
  ReactiveParallelNode & operator=(const ReactiveParallelNode &) = delete;
  ReactiveParallelNode(ReactiveParallelNode &&) = delete;
  ReactiveParallelNode & operator=(ReactiveParallelNode &&) = delete;

  void halt() override;

  size_t successThreshold() const;
  size_t failureThreshold() const;
  void setSuccessThreshold(int threshold);
  void setFailureThreshold(int threshold);

private:
  int success_threshold_;
  int failure_threshold_;

  bool read_parameter_from_ports_;
  static constexpr const char * THRESHOLD_SUCCESS = "success_count";
  static constexpr const char * THRESHOLD_FAILURE = "failure_count";

  BT::NodeStatus tick() override;
};

}  // namespace combat_sentry_behavior
