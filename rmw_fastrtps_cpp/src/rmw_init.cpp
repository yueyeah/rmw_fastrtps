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

#include "rcutils/types.h"

#include "rmw/impl/cpp/macros.hpp"
#include "rmw/init.h"
#include "rmw/init_options.h"
#include "rmw/rmw.h"

#include "rmw_dds_common/context.hpp"
#include "rmw_dds_common/msg/participant_entities_info.hpp"

#include "rmw_fastrtps_shared_cpp/custom_participant_info.hpp"
#include "rmw_fastrtps_shared_cpp/participant.hpp"
#include "rmw_fastrtps_shared_cpp/publisher.hpp"
#include "rmw_fastrtps_shared_cpp/rmw_context_impl.h"
#include "rmw_fastrtps_shared_cpp/subscription.hpp"

#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include "rmw_fastrtps_cpp/identifier.hpp"
#include "rmw_fastrtps_cpp/publisher.hpp"
#include "rmw_fastrtps_cpp/subscription.hpp"

#include "listener_thread.hpp"

extern "C"
{
rmw_ret_t
rmw_init_options_init(rmw_init_options_t * init_options, rcutils_allocator_t allocator)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&allocator, return RMW_RET_INVALID_ARGUMENT);
  if (NULL != init_options->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected zero-initialized init_options");
    return RMW_RET_INVALID_ARGUMENT;
  }
  init_options->instance_id = 0;
  init_options->implementation_identifier = eprosima_fastrtps_identifier;
  init_options->allocator = allocator;
  init_options->impl = nullptr;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_copy(const rmw_init_options_t * src, rmw_init_options_t * dst)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(src, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(dst, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    src,
    src->implementation_identifier,
    eprosima_fastrtps_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (NULL != dst->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected zero-initialized dst");
    return RMW_RET_INVALID_ARGUMENT;
  }
  *dst = *src;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_fini(rmw_init_options_t * init_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&(init_options->allocator), return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    init_options,
    init_options->implementation_identifier,
    eprosima_fastrtps_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  *init_options = rmw_get_zero_initialized_init_options();
  return RMW_RET_OK;
}

using rmw_dds_common::msg::ParticipantEntitiesInfo;

rmw_ret_t
rmw_init(const rmw_init_options_t * options, rmw_context_t * context)
{
  rmw_dds_common::Context * common_context = nullptr;
  CustomParticipantInfo * participant_info = nullptr;
  rmw_publisher_t * publisher = nullptr;
  rmw_ret_t ret = RMW_RET_OK;
  rmw_subscription_t * subscription = nullptr;
  rmw_qos_profile_t qos = rmw_qos_profile_default;

  RCUTILS_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    options,
    options->implementation_identifier,
    eprosima_fastrtps_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  context->instance_id = options->instance_id;
  context->implementation_identifier = eprosima_fastrtps_identifier;

  context->impl = static_cast<rmw_context_impl_t *>(rmw_allocate(sizeof(rmw_context_impl_t)));
  if (nullptr == context->impl) {
    ret = RMW_RET_BAD_ALLOC;
    goto fail;
  }

  common_context =
    static_cast<rmw_dds_common::Context *>(rmw_allocate(sizeof(rmw_dds_common::Context)));
  if (nullptr == common_context) {
    goto fail;
  }
  RMW_TRY_PLACEMENT_NEW(
    common_context,
    common_context,
    ret = RMW_RET_BAD_ALLOC; goto fail,
    rmw_dds_common::Context, );
  context->impl->common = static_cast<void *>(common_context);

  participant_info = rmw_fastrtps_shared_cpp::create_participant(
    eprosima_fastrtps_identifier,
    options->domain_id,
    &options->security_options,
    common_context);
  if (nullptr == participant_info) {
    ret = RMW_RET_BAD_ALLOC;
    goto fail;
  }
  context->impl->participant_info = static_cast<void *>(participant_info);

  qos.avoid_ros_namespace_conventions = false;  // Change it to true after testing
  qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos.depth = 1;
  qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

  publisher = rmw_fastrtps_cpp::create_publisher(
    participant_info,
    rosidl_typesupport_cpp::get_message_type_support_handle<ParticipantEntitiesInfo>(),
    "_participant_info",
    &qos,
    false,  // our fastrtps typesupport doesn't support keyed topics
    true);  // don't create a publisher listener
  if (nullptr == publisher) {
    ret = RMW_RET_BAD_ALLOC;
    goto fail;
  }
  // using same qos for the subscription
  subscription = rmw_fastrtps_cpp::create_subscription(
    participant_info,
    rosidl_typesupport_cpp::get_message_type_support_handle<ParticipantEntitiesInfo>(),
    "_participant_info",
    &qos,
    true,  // ignore_local_publications, currently not implemented
    false,  // our fastrtps typesupport doesn't support keyed topics
    true);  // don't create a subscriber listener
  if (nullptr == subscription) {
    ret = RMW_RET_BAD_ALLOC;
    goto fail;
  }
  common_context->gid = rmw_fastrtps_shared_cpp::create_rmw_gid(
    eprosima_fastrtps_identifier, participant_info->participant->getGuid());
  common_context->pub = publisher;
  common_context->sub = subscription;
  // This is not more needed.
  // node_cache.add_gid(common_context->gid);
  common_context->graph_cache.add_participant(common_context->gid);

  ret = rmw_fastrtps_cpp::run_listener_thread(context);
  if (RMW_RET_OK != ret) {
    goto fail;
  }
  return RMW_RET_OK;
fail:
  if (common_context) {
    ret = rmw_fastrtps_cpp::join_listener_thread(context);
    if (RMW_RET_OK != ret) {
      return ret;
    }
    if (common_context->pub) {
      ret = rmw_fastrtps_shared_cpp::destroy_publisher(
        eprosima_fastrtps_identifier,
        participant_info,
        common_context->pub);
      if (RMW_RET_OK != ret) {
        return ret;
      }
    }

    if (common_context->sub) {
      ret = rmw_fastrtps_shared_cpp::destroy_subscription(
        eprosima_fastrtps_identifier,
        participant_info,
        common_context->sub);
      if (RMW_RET_OK != ret) {
        return ret;
      }
    }
  }

  if (participant_info) {
    ret = rmw_fastrtps_shared_cpp::destroy_participant(participant_info);
    if (RMW_RET_OK != ret) {
      return ret;
    }
  }
  rmw_free(context->impl->common);
  *context = rmw_get_zero_initialized_context();
  return ret;
}

rmw_ret_t
rmw_shutdown(rmw_context_t * context)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    eprosima_fastrtps_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // Nothing to do here for now.
  // This is just the middleware's notification that shutdown was called.
  return RMW_RET_OK;
}

rmw_ret_t
rmw_context_fini(rmw_context_t * context)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    eprosima_fastrtps_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  rmw_ret_t ret = rmw_fastrtps_cpp::join_listener_thread(context);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  auto participant_info = static_cast<CustomParticipantInfo *>(context->impl->participant_info);
  auto common_context = static_cast<rmw_dds_common::Context *>(context->impl->common);
  ret = rmw_fastrtps_shared_cpp::destroy_publisher(
    eprosima_fastrtps_identifier,
    participant_info,
    common_context->pub);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  ret = rmw_fastrtps_shared_cpp::destroy_subscription(
    eprosima_fastrtps_identifier,
    participant_info,
    common_context->sub);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  ret = rmw_fastrtps_shared_cpp::destroy_participant(participant_info);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  rmw_free(context->impl->common);

  *context = rmw_get_zero_initialized_context();
  return RMW_RET_OK;
}
}  // extern "C"
