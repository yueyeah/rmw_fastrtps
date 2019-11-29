// Copyright 2019 Open Source Robotics Foundation, Inc.
// Copyright 2016-2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <string>

#include "rcutils/allocator.h"

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/get_topic_names_and_types.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/names_and_types.h"
#include "rmw/types.h"

#include "rmw_dds_common/context.hpp"

#include "rmw_fastrtps_shared_cpp/rmw_common.hpp"
#include "rmw_fastrtps_shared_cpp/rmw_context_impl.h"

#include "demangle.hpp"

namespace rmw_fastrtps_shared_cpp
{

using DemangleFunction = std::string (*)(const std::string &);

rmw_ret_t
__rmw_get_topic_names_and_types(
  const char * identifier,
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  if (!allocator) {
    RMW_SET_ERROR_MSG("allocator is null");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (!node) {
    RMW_SET_ERROR_MSG("null node handle");
    return RMW_RET_INVALID_ARGUMENT;
  }
  rmw_ret_t ret = rmw_names_and_types_check_zero(topic_names_and_types);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  if (node->implementation_identifier != identifier) {
    RMW_SET_ERROR_MSG("node handle not from this implementation");
    return RMW_RET_ERROR;
  }

  DemangleFunction demangle_topic = _demangle_ros_topic_from_topic;
  DemangleFunction demangle_type = _demangle_if_ros_type;
  DemangleFunction no_op{[](const std::string & x) {return x;}};

  if (no_demangle) {
    demangle_topic = no_op;
    demangle_type = no_op;
  }
  auto common_context = static_cast<rmw_dds_common::Context *>(node->context->impl->common);

  return common_context->graph_cache.get_names_and_types(
    demangle_topic,
    demangle_type,
    allocator,
    topic_names_and_types);
}
}  // namespace rmw_fastrtps_shared_cpp
