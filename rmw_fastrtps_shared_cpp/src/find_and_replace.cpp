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

#include <string>

std::string find_and_replace(const std::string input, const std::string find, const std::string replace) {
  const std::size_t find_len = find.length();
  std::string output = input;
  std::size_t pos = output.find(find);
  while (pos != std::string::npos) {
    output.replace(pos, find_len, replace);
    pos = output.find(find);
  }
  return output;
}
