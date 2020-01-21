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
  unsigned char * hmac,
  unsigned int * hmac_size)
{
  char fn_id[] = "create_hmac";
  printf("Just entered %s\n", fn_id);
  unsigned char * pre_hmac_ros_message = (unsigned char *) ros_message;
  size_t key_len = strlen(key);
  size_t message_len = strlen((const char *)pre_hmac_ros_message);
  // Computing hmac using OpenSSL Hmac function, put computed hmac into hmac and the length of the computed hmac into hmac_size
  HMAC(EVP_md5(), key, key_len, pre_hmac_ros_message, message_len, hmac, hmac_size);
  printf("%s: computed hmac_size: %d\n", fn_id, *hmac_size);
  printf("%s: computed hmac: %x\n", fn_id, *hmac);
  return true; 
}

/**
 * Extracts the hmac from the ros_message and compares it against the hmac 
 * computed from the ros_message 
 */
bool
is_hmac_matched(
  const char * key,
  void * deser_ros_message)
{
  char fn_id[] = "is_hmac_matched";
  fprintf(stdout, "Just entered %s fn\n", fn_id);
  size_t key_len = strlen(key);
  
  // casting and parsing into hmac_size, hmac and ros_message
  const char * cast_deser_ros_message = (const char *)deser_ros_message;
  size_t cast_deser_ros_message_len = strlen(cast_deser_ros_message);
  // get size of hmac which is the first index in the deserialized message
  size_t received_hmac_size = (size_t) cast_deser_ros_message[0];
  // put the received hmac into received_hmac
  unsigned char * received_hmac = (unsigned char *)malloc(received_hmac_size);
  memcpy(received_hmac, cast_deser_ros_message+1, received_hmac_size);
  // put the received message into received_ros_message
  size_t received_ros_message_size = cast_deser_ros_message_len - 1 - received_hmac_size;
  unsigned char * received_ros_message = (unsigned char *)malloc(received_ros_message_size);
  memcpy(received_ros_message, cast_deser_ros_message+1+received_hmac_size, received_ros_message_size);

  unsigned char * computed_hmac = (unsigned char *)malloc(sizeof(unsigned char)); 
  HMAC(EVP_md5(), key, key_len, received_ros_message, received_ros_message_size, computed_hmac, NULL);
  fprintf(stdout, "%s: hmac computed from received message, now comparing with received hmac\n", fn_id);
  fprintf(stdout, "%s: computed hmac is: %x\n", fn_id, *computed_hmac);
  fprintf(stdout, "%s: received hmac is: %x\n", fn_id, *received_hmac);
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
  unsigned char * hmac = (unsigned char *)malloc(sizeof(unsigned char));
  unsigned int * hmac_size = (unsigned int *)malloc(sizeof(unsigned int));
  const char * key = (const char *) "01234567890"; // key will be put somewhere else in the future
  if (!create_hmac(ros_message, key, hmac, hmac_size)) {
    RMW_SET_ERROR_MSG("unable to compute hmac for message");
    return RMW_RET_ERROR;
  }
  printf("%s: hmac for message: %x\n", fn_id, *hmac);

  // new_ros_message = (char)hmac_size + hmac + (char)ros_message
  // This new_ros_message will be serialized instead of the existing 
  // implementation where ros_message is serialized 
  unsigned char * cast_ros_message = (unsigned char *)ros_message;
  printf("%s cast_ros_message: %s\n", fn_id, cast_ros_message);
  unsigned char * cast_hmac_size = (unsigned char *)hmac_size;
  printf("%s cast_hmac_size: %s\n", fn_id, cast_hmac_size);
  unsigned char * new_ros_message = (unsigned char *)malloc(sizeof(*cast_hmac_size) + sizeof(*hmac) + (sizeof(*cast_ros_message)));
  memcpy(new_ros_message, cast_hmac_size, sizeof(*cast_hmac_size));
  memcpy(new_ros_message, hmac, sizeof(*hmac));
  memcpy(new_ros_message, cast_ros_message, sizeof(*cast_ros_message));
  printf("%s new_ros_message: %send\n", fn_id, new_ros_message);

  auto callbacks = static_cast<const message_type_support_callbacks_t *>(ts->data);
  auto tss = new MessageTypeSupport_cpp(callbacks);
  auto data_length = tss->getEstimatedSerializedSize(new_ros_message);
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
  free(hmac_size);
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
  if (!is_hmac_matched(key, ros_message)) {
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
