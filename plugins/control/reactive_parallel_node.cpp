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

#include "combat_sentry_behavior/plugins/control/reactive_parallel_node.hpp"

#include <algorithm>
#include <cstddef>

namespace combat_sentry_behavior
{
constexpr const char * ReactiveParallelNode::THRESHOLD_FAILURE;
constexpr const char * ReactiveParallelNode::THRESHOLD_SUCCESS;

ReactiveParallelNode::ReactiveParallelNode(const std::string & name)
: BT::ControlNode::ControlNode(name, {}),
  success_threshold_(-1),
  failure_threshold_(1),
  read_parameter_from_ports_(false)
{
  setRegistrationID("ReactiveParallel");
}

ReactiveParallelNode::ReactiveParallelNode(
  const std::string & name, const BT::NodeConfig & config)
: BT::ControlNode::ControlNode(name, config),
  success_threshold_(-1),
  failure_threshold_(1),
  read_parameter_from_ports_(true)
{
}

BT::NodeStatus ReactiveParallelNode::tick()
{
  if (read_parameter_from_ports_) {
    if (!getInput(THRESHOLD_SUCCESS, success_threshold_)) {
      throw BT::RuntimeError("Missing parameter [", THRESHOLD_SUCCESS, "] in ReactiveParallel");
    }

    if (!getInput(THRESHOLD_FAILURE, failure_threshold_)) {
      throw BT::RuntimeError("Missing parameter [", THRESHOLD_FAILURE, "] in ReactiveParallel");
    }
  }

  const size_t children_count = children_nodes_.size();

  if (children_count < successThreshold()) {
    throw BT::LogicError("Number of children is less than threshold. Can never succeed.");
  }

  if (children_count < failureThreshold()) {
    throw BT::LogicError("Number of children is less than threshold. Can never fail.");
  }

  setStatus(BT::NodeStatus::RUNNING);

  size_t skipped_count = 0;
  size_t current_success_count = 0;
  size_t current_failure_count = 0;

  for (size_t i = 0; i < children_count; i++) {
    BT::TreeNode * child_node = children_nodes_[i];
    const BT::NodeStatus child_status = child_node->executeTick();

    switch (child_status) {
      case BT::NodeStatus::SKIPPED: {
        skipped_count++;
      } break;

      case BT::NodeStatus::SUCCESS: {
        current_success_count++;
      } break;

      case BT::NodeStatus::FAILURE: {
        current_failure_count++;
      } break;

      case BT::NodeStatus::RUNNING: {
        break;
      }

      case BT::NodeStatus::IDLE: {
        throw BT::LogicError("[", name(), "]: A child should not return IDLE");
      }
    }

    const size_t required_success_count = successThreshold();

    if (
      current_success_count >= required_success_count ||
      (success_threshold_ < 0 &&
       (current_success_count + skipped_count) >= required_success_count)) {
      resetChildren();
      return BT::NodeStatus::SUCCESS;
    }

    // It fails if it is not possible to succeed anymore or if
    // number of failures are equal to failure_threshold_
    if (
      ((children_count - current_failure_count) < required_success_count) ||
      (current_failure_count == failureThreshold())) {
      resetChildren();
      return BT::NodeStatus::FAILURE;
    }
  }

  // Skip if ALL the nodes have been skipped
  return (skipped_count == children_count) ? BT::NodeStatus::SKIPPED : BT::NodeStatus::RUNNING;
}

void ReactiveParallelNode::halt()
{
  BT::ControlNode::halt();
}

size_t ReactiveParallelNode::successThreshold() const
{
  if (success_threshold_ < 0) {
    return size_t(std::max(int(children_nodes_.size()) + success_threshold_ + 1, 0));
  } else {
    return size_t(success_threshold_);
  }
}

size_t ReactiveParallelNode::failureThreshold() const
{
  if (failure_threshold_ < 0) {
    return size_t(std::max(int(children_nodes_.size()) + failure_threshold_ + 1, 0));
  } else {
    return size_t(failure_threshold_);
  }
}

void ReactiveParallelNode::setSuccessThreshold(int threshold)
{
  success_threshold_ = threshold;
}

void ReactiveParallelNode::setFailureThreshold(int threshold)
{
  failure_threshold_ = threshold;
}

}  // namespace combat_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<combat_sentry_behavior::ReactiveParallelNode>("ReactiveParallel");
}
