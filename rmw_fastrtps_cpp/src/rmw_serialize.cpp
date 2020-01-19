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

#include "fastcdr/FastBuffer.h"

#include "rmw/error_handling.h"
#include "rmw/serialized_message.h"
#include "rmw/rmw.h"

#include "./type_support_common.hpp"
#include <stdio.h>
#include <string.h>
#include <openssl/hmac.h>
#include <stdbool.h>

extern "C"
{

/**
 * Computes hmac based on the ros_message then adds it to the hmac field of the serialized message 
 */
rmw_ret_t
create_hmac(
  const void * ros_message,
  const char * key,
  rmw_serialized_message_t * serialized_message)
{
  unsigned char * pre_hmac_ros_message = (unsigned char *) ros_message;
  size_t key_len = strlen(key);
  size_t message_len = strlen((const char *)pre_hmac_ros_message);
  unsigned int * hmac_size;
  unsigned char * hmac;
  HMAC(EVP_md5(), key, key_len, pre_hmac_ros_message, message_len, hmac, hmac_size);
  rmw_ret_t ret = rcutils_uint8_array_hmac_init(serialized_message, (size_t)hmac_size);
  serialized_message->hmac = hmac;
  printf("Sending hmac: %s\n", serialized_message->hmac);
  return ret; 
}

/**
 * Computes a new hmac based on the ros_message then compares with the hmac already stored in the 
 * hmac field of the serialized message 
 */
bool
is_hmac_matched(
  const char * key,
  const rmw_serialized_message_t * serialized_message)
{
  fprintf(stdout, "Just entered is_hmac_matched fn\n");
  unsigned char * received_ros_message = (unsigned char *) serialized_message->buffer;
  size_t key_len = strlen(key);
  size_t message_len = strlen((const char *)received_ros_message);
  unsigned char * computed_hmac; /* stores the hmac computed from the received message */
  HMAC(EVP_md5(), key, key_len, received_ros_message, message_len, computed_hmac, NULL);
  const char * received_hmac;
  fprintf(stdout, "hmac computed from received message, now comparing with received hmac\n");
  fprintf(stdout, "computed hmac is: %s\n", computed_hmac);
  fprintf(stdout, "received hmac is: %s\n", received_hmac);
  return strcmp((const char *)computed_hmac, received_hmac) == 0; /* returns true if hmacs are equal, false otherwise */
}

/**
 * Increments the counter of the serialized message by 1
 */
/*
bool 
increment_counter(
  rmw_serialized_message_t * serialized_message)
{
  if (serialized_message->counter == 0) {
    serialized_message->counter = rand()
  } else {
    serialized_message->counter++;
  }
}
*/

/**
 * Checks that the counter is incremented correctly.
 */
/*
bool is_counter_inc_correctly(
  rmw_serialized_message_t * serialized_message)
{
  if 
}
*/

rmw_ret_t
rmw_serialize(
  const void * ros_message,
  const rosidl_message_type_support_t * type_support,
  rmw_serialized_message_t * serialized_message)
{
  const rosidl_message_type_support_t * ts = get_message_typesupport_handle(
    type_support, RMW_FASTRTPS_CPP_TYPESUPPORT_C);
  if (!ts) {
    ts = get_message_typesupport_handle(
      type_support, RMW_FASTRTPS_CPP_TYPESUPPORT_CPP);
    if (!ts) {
      RMW_SET_ERROR_MSG("type support not from this implementation");
      return RMW_RET_ERROR;
    }
  }
  // assert(type support checked)

  // insert create_hmac here. add hmac to the ros_message before resizing/serialising
  // key must put somewhere else
  const char * key = (const char *) "01234567890";
  if (!create_hmac(ros_message, key, serialized_message)) {
    RMW_SET_ERROR_MSG("unable to compute hmac for message");
    return RMW_RET_ERROR;
  }

  auto callbacks = static_cast<const message_type_support_callbacks_t *>(ts->data);
  auto tss = new MessageTypeSupport_cpp(callbacks);
  auto data_length = tss->getEstimatedSerializedSize(ros_message);
  if (serialized_message->buffer_capacity < data_length) {
    if (rmw_serialized_message_resize(serialized_message, data_length) != RMW_RET_OK) {
      RMW_SET_ERROR_MSG("unable to dynamically resize serialized message");
      return RMW_RET_ERROR;
    }
  }
  // assert(able to dynamically resize serialised message if message bigger than buffer)
  // assert(serialised message can fit in buffer)

  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), data_length);
  eprosima::fastcdr::Cdr ser(
    buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);

  auto ret = tss->serializeROSmessage(ros_message, ser);
  serialized_message->buffer_length = data_length;
  serialized_message->buffer_capacity = data_length;
  delete tss;
  return ret == true ? RMW_RET_OK : RMW_RET_ERROR;
}

rmw_ret_t
rmw_deserialize(
  const rmw_serialized_message_t * serialized_message,
  const rosidl_message_type_support_t * type_support,
  void * ros_message)
{
  fprintf(stdout, "Just entered rmw_deserialize fn\n");
  printf("serialized_message hmac is: %s\n", serialized_message->hmac);
  const rosidl_message_type_support_t * ts = get_message_typesupport_handle(
    type_support, RMW_FASTRTPS_CPP_TYPESUPPORT_C);
  if (!ts) {
    ts = get_message_typesupport_handle(
      type_support, RMW_FASTRTPS_CPP_TYPESUPPORT_CPP);
    if (!ts) {
      RMW_SET_ERROR_MSG("type support not from this implementation\n");
      return RMW_RET_ERROR;
    }
  }
  fprintf(stdout, "Checked rmw_serialize message_typesupport\n");

  auto callbacks = static_cast<const message_type_support_callbacks_t *>(ts->data);
  auto tss = new MessageTypeSupport_cpp(callbacks);
  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), serialized_message->buffer_length);
  eprosima::fastcdr::Cdr deser(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
    eprosima::fastcdr::Cdr::DDS_CDR);

  auto ret = tss->deserializeROSmessage(deser, ros_message);
  fprintf(stdout, "Serialized message is deserialized\n");

  // key must put somewhere else
  const char * key = (const char *) "01234567890";
  if (!is_hmac_matched(key, serialized_message)) {
    fprintf(stdout, "hmac does not match, possible alteration/interception of messages\n");
    RMW_SET_ERROR_MSG("hmac does not match, possible alteration/interception of messages");
    return RMW_RET_ERROR;
  }
  fprintf(stdout, "hmac matches, no unauthorised alteration of messages\n");

  delete tss;
  return ret == true ? RMW_RET_OK : RMW_RET_ERROR;
}

rmw_ret_t
rmw_get_serialized_message_size(
  const rosidl_message_type_support_t * /*type_support*/,
  const rosidl_message_bounds_t * /*message_bounds*/,
  size_t * /*size*/)
{
  RMW_SET_ERROR_MSG("unimplemented");
  return RMW_RET_ERROR;
}

}  // extern "C"
