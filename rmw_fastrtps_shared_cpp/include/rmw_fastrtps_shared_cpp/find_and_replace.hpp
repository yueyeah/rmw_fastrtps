// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RMW_FASTRTPS_SHARED_CPP__FIND_AND_REPLACE_HPP_
#define RMW_FASTRTPS_SHARED_CPP__FIND_AND_REPLACE_HPP_

#include <string>

#include "./visibility_control.h"

RMW_FASTRTPS_SHARED_CPP_PUBLIC
std::string
find_and_replace(const std::string input, const std::string find, const std::string replace);

#endif  // RMW_FASTRTPS_SHARED_CPP__FIND_AND_REPLACE_HPP_
