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
 * Serialize a ROS message into a rmw_serialized_message_t. ROS message is serialized into
 * a byte stream contained within rmw_serialized_message_t structure. Serialization format 
 * depends on underlying middleware. 
 * \@param ros_message The typed ROS message.
 * \@param type_support The typesupport for the ROS message.
 * \@param serialized_message The destination for the serialize ROS message. 
 */
rmw_ret_t
rmw_serialize(
  const void * ros_message,
  const rosidl_message_type_support_t * type_support,
  rmw_serialized_message_t * serialized_message)
{
  char fn_id[] = "rmw_serialize";
  printf("Just entered %s\n", fn_id);
  
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
  printf("%s: type support is checked\n", fn_id);

  auto callbacks = static_cast<const message_type_support_callbacks_t *>(ts->data);
  auto tss = new MessageTypeSupport_cpp(callbacks);
  auto original_data_length = tss->getEstimatedSerializedSize(ros_message);
  auto data_length = original_data_length + 17;
  if (serialized_message->buffer_capacity < data_length) {
    if (rmw_serialized_message_resize(serialized_message, data_length) != RMW_RET_OK) {
      RMW_SET_ERROR_MSG("unable to dynamically resize serialized message");
      return RMW_RET_ERROR;
    }
  }
  printf("%s: serialized message can fit in buffer.\n", fn_id);

  // Serializing the ros_message
  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), data_length);
  eprosima::fastcdr::Cdr ser(
    buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);
  auto ret = tss->serializeROSmessage(ros_message, ser);
  serialized_message->buffer_length = data_length;
  serialized_message->buffer_capacity = data_length;

  unsigned char * serialized_orig_ros_msg = (unsigned char *)malloc(original_data_length);
  memcpy(serialized_orig_ros_msg, serialized_message->buffer, original_data_length);
  // serialized_orig_ros_msg now contains serialized original ros message.

  const char * key = "01234567890";
  unsigned char * hmac = (unsigned char *)malloc(16);
  HMAC(EVP_md5(), key, 11, serialized_orig_ros_msg, original_data_length, hmac, NULL);
  memcpy(serialized_message->buffer+(original_data_length), hmac, 16);
  // the hmac is obtained from serialized_orig_ros_msg and appended to the end 
  // of the serialized original ros message
  
  // rand_int_counter gets the counter passed to it via
  // serialized_message->counter field, and makes it the last
  // byte in the buffer to ensure safe transmission to other node
  unsigned char * rand_int_counter = (unsigned char *)&serialized_message->counter;
  memcpy(serialized_message->buffer+(original_data_length + 16), rand_int_counter, 1);
  printf("%s: rand_int_counter: %d\n", fn_id, *rand_int_counter);

  // debug statement to find out the contents of the serialized message buffer 
  // while in the rmw_serialize fn
  printf("%s: serialized buffer: ", fn_id);
  for (size_t i = 0; i < serialized_message->buffer_length; ++i) {
    printf("%02x ", serialized_message->buffer[i]);
  }
  printf("\n");
  printf("%s: serialized data_length: %ldend\n", fn_id, data_length);

  delete tss;

  // Clean up everything that I have malloced
  free(hmac);

  return ret == true ? RMW_RET_OK : RMW_RET_ERROR;
}

/**
 * Deserialize a ROS message. The given rmw_serialized_message_t's internal byte stream 
 * buffer is deserialized into given ROS message. The ROS message must already be allocated
 * and initialized, and must match the given typesupport structure. The serialization 
 * format expected in the rmw_serialized_message_t depends on the underlying middleware. 
 * \@param serialized_message The serialized message holding the byte stream. 
 * \@param type_support The typesupport for the typed ros message. 
 * \@param ros_message Destination for the deserialized ROS message. 
 */
rmw_ret_t
rmw_deserialize(
  const rmw_serialized_message_t * serialized_message,
  const rosidl_message_type_support_t * type_support,
  void * ros_message)
{
  char fn_id[] = "rmw_deserialize";
  fprintf(stdout, "Just entered %s\n", fn_id);
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

  // debug statement in rmw_deserialize, print out buffer before deserializing
  printf("%s: serialized buffer: ", fn_id);
  for (size_t i = 0; i < serialized_message->buffer_length; ++i) {
    printf("%02x ", serialized_message->buffer[i]);
  }
  printf("\n");

  const char * key = (const char *) "01234567890";

  // 19 = (length of hmac which is 16) + (1 rand_int counter byte) + (2 extra bytes at end)
  int original_ros_msg_length = serialized_message->buffer_length - 18;
  unsigned char * original_ros_msg_serialized = (unsigned char *)malloc(original_ros_msg_length);
  memcpy(original_ros_msg_serialized, serialized_message->buffer, original_ros_msg_length);
  unsigned char * computed_hmac = (unsigned char *)malloc(16);
  HMAC(EVP_md5(), key, 11, original_ros_msg_serialized, original_ros_msg_length, computed_hmac, NULL);
  // computed_hmac now contains the hmac computed from the original ros 
  // message in serialized form

  // extract the rand_int counter from the third last byte of the buffer,
  // insert into serialized_message->counter field to transfer to subscriber node
  // int rand_int_counter = (int)serialized_message->buffer[original_ros_msg_length + 16];
  // serialized_message->counter = rand_int_counter;
  // THIS CANNOT BE CARRIED OUT AS serialized_message IS const

  unsigned char * received_hmac = (unsigned char *)malloc(16);
  memcpy(received_hmac, serialized_message->buffer + original_ros_msg_length, 16);
  // received_hmac now contains the last 16 bytes i.e. the received hmac

  if (strncmp((const char *)computed_hmac, (const char *)received_hmac, 16) == 0) {
    printf("%s: hmac are equal\n", fn_id);
  } else {
    printf("%s: hmac are not equal, possible interception/alteration\n", fn_id);
  }

  auto callbacks = static_cast<const message_type_support_callbacks_t *>(ts->data);
  auto tss = new MessageTypeSupport_cpp(callbacks);
  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), serialized_message->buffer_length);
  eprosima::fastcdr::Cdr deser(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
    eprosima::fastcdr::Cdr::DDS_CDR);

  auto ret = tss->deserializeROSmessage(deser, ros_message);
  fprintf(stdout, "%s: deserialized_message: %send\n", fn_id, (char *)ros_message);
  delete tss;

  // Clean up everything that I have malloced
  free(original_ros_msg_serialized);
  free(computed_hmac);
  free(received_hmac);

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
