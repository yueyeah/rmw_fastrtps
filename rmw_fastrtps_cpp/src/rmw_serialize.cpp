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
 * Computes hmac based on the ros_message. Put the length of the hmac into hmac_size.
 */
bool
create_hmac(
  const void * ros_message,
  const char * key,
  unsigned char * hmac)
{
  char fn_id[] = "create_hmac";
  printf("Just entered %s\n", fn_id);
  unsigned char * pre_hmac_ros_message = (unsigned char *) ros_message;
  size_t key_len = strlen(key);
  size_t message_len = strlen((const char *)pre_hmac_ros_message);
  // Computing hmac using OpenSSL Hmac function, put computed hmac into hmac and the length of the computed hmac into hmac_size
  HMAC(EVP_md5(), key, key_len, pre_hmac_ros_message, message_len, hmac, NULL);
  printf("%s: computed hmac: %s\n", fn_id, hmac);
  return true; 
}

/**
 * Extracts the hmac from the ros_message and compares it against the hmac 
 * computed from the ros_message 
 */
bool
is_hmac_matched(
  const char * key,
  void * deser_ros_message,
  size_t deser_ros_message_len)
{
  char fn_id[] = "is_hmac_matched";
  fprintf(stdout, "Just entered %s fn\n", fn_id);
  size_t key_len = strlen(key);

  // casting and parsing into hmac_size, hmac and ros_message
  const char * cast_deser_ros_message = (const char *)deser_ros_message;

  // put the received hmac into received_hmac variable
  unsigned char * received_hmac = (unsigned char *)malloc(16);
  memcpy(received_hmac, cast_deser_ros_message, 16);
  printf("%s: received_hmac: %send\n", fn_id, received_hmac);

  // put the received message into received_ros_message
  size_t received_ros_message_size = deser_ros_message_len - 16;
  unsigned char * received_ros_message = (unsigned char *)malloc(received_ros_message_size);
  memcpy(received_ros_message, cast_deser_ros_message + 16, deser_ros_message_len);
  printf("%s: received_ros_message: %send\n", fn_id, received_ros_message);

  // compute hmac and compare with received_hmac
  unsigned char * computed_hmac = (unsigned char *)malloc(16); 
  HMAC(EVP_md5(), key, key_len, received_ros_message, received_ros_message_size, computed_hmac, NULL);
  printf("%s: computed_hmac: %send\n", fn_id, computed_hmac);
  return strcmp((const char *)computed_hmac, (const char *)received_hmac) == 0; /* returns true if hmacs are equal, false otherwise */
}

rmw_ret_t
rmw_serialize(
  const void * ros_message,
  const rosidl_message_type_support_t * type_support,
  rmw_serialized_message_t * serialized_message)
{
  char fn_id[] = "rmw_serialize";
  printf("\nJust entered %s\n", fn_id);
  
  printf("%s: ros_message is %send\n", fn_id, (const char *) ros_message);
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
  printf("%s: type support is checked\n", fn_id);

  // Create hmac before the message is resized and serialized. 
  // Remember to free hmac and hmac_size after usage!!
  unsigned char * hmac = (unsigned char *)malloc(16);
  const char * key = (const char *) "01234567890"; // key will be put somewhere else in the future
  if (!create_hmac(ros_message, key, hmac)) {
    RMW_SET_ERROR_MSG("unable to compute hmac for message");
    return RMW_RET_ERROR;
  }
  printf("%s: hmac for message: %s\n", fn_id, hmac);

  // new_ros_message = hmac + (char)ros_message
  // This new_ros_message will be serialized instead of the existing 
  // implementation where ros_message is serialized 
  unsigned char * cast_ros_message = (unsigned char *)ros_message;
  printf("%s: cast_ros_message: %s\n", fn_id, cast_ros_message);
  unsigned char * new_ros_message = (unsigned char *)malloc(sizeof(hmac) + (sizeof(cast_ros_message)));
  memcpy(new_ros_message, hmac, 16);
  memcpy(new_ros_message, cast_ros_message, sizeof(cast_ros_message));
  printf("%s: new_ros_message: %send\n", fn_id, new_ros_message);

  auto callbacks = static_cast<const message_type_support_callbacks_t *>(ts->data);
  auto tss = new MessageTypeSupport_cpp(callbacks);
  auto data_length = tss->getEstimatedSerializedSize(ros_message) + 16;
  if (serialized_message->buffer_capacity < data_length) {
    if (rmw_serialized_message_resize(serialized_message, data_length) != RMW_RET_OK) {
      RMW_SET_ERROR_MSG("unable to dynamically resize serialized message");
      return RMW_RET_ERROR;
    }
  }
  // assert(able to dynamically resize serialised message if message bigger than buffer)
  // assert(serialised message can fit in buffer)
  printf("%s: serialized message can fit in buffer.\n", fn_id);

  // Serializing the new_ros_message
  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), data_length);
  eprosima::fastcdr::Cdr ser(
    buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);
  auto ret = tss->serializeROSmessage(new_ros_message, ser);
  serialized_message->buffer_length = data_length;
  serialized_message->buffer_capacity = data_length;
  printf("%s: serialized data_length: %ldend\n", fn_id, data_length);

  delete tss;

  // Clean up everything that I have malloced
  free(new_ros_message);
  free(hmac);

  return ret == true ? RMW_RET_OK : RMW_RET_ERROR;
}

rmw_ret_t
rmw_deserialize(
  const rmw_serialized_message_t * serialized_message,
  const rosidl_message_type_support_t * type_support,
  void * ros_message)
{
  char fn_id[] = "rmw_deserialize";
  fprintf(stdout, "\nJust entered %s\n", fn_id);
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
  fprintf(stdout, "%s: checked message_typesupport\n", fn_id);

  auto callbacks = static_cast<const message_type_support_callbacks_t *>(ts->data);
  auto tss = new MessageTypeSupport_cpp(callbacks);
  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), serialized_message->buffer_length);
  eprosima::fastcdr::Cdr deser(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
    eprosima::fastcdr::Cdr::DDS_CDR);

  auto ret = tss->deserializeROSmessage(deser, ros_message);
  fprintf(stdout, "%s: serialized message is deserialized\n", fn_id);

  // key must put somewhere else
  const char * key = (const char *) "01234567890";
  if (!is_hmac_matched(key, ros_message, serialized_message->buffer_length)) {
    fprintf(stdout, "%s: hmac does not match, possible alteration/interception of messages\n", fn_id);
    RMW_SET_ERROR_MSG("hmac does not match, possible alteration/interception of messages");
    return RMW_RET_ERROR;
  }
  fprintf(stdout, "%s: hmac matches, no unauthorised alteration of messages\n", fn_id);

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
