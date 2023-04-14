// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef OCC_GRIDMAP__PROTO_IO_HPP_
#define OCC_GRIDMAP__PROTO_IO_HPP_

#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <fstream>
#include <string>

#include "google/protobuf/message.h"

namespace io
{

class ProtoStreamWriter
{
public:
  explicit ProtoStreamWriter(const std::string & filename);
  ~ProtoStreamWriter() = default;

  ProtoStreamWriter(const ProtoStreamWriter &) = delete;
  ProtoStreamWriter & operator=(const ProtoStreamWriter &) = delete;

  void WriteProto(const google::protobuf::Message & proto);

  bool Close();

private:
  void Write(const std::string & uncompressed_data);

private:
  std::ofstream out_;
};  // class ProtoStreamWriter

class ProtoStreamReader
{
public:
  explicit ProtoStreamReader(const std::string & filename);
  ~ProtoStreamReader() = default;

  ProtoStreamReader(const ProtoStreamReader &) = delete;
  ProtoStreamReader & operator=(const ProtoStreamReader &) = delete;

  bool ReadProto(google::protobuf::Message * proto);
  bool eof() const;

private:
  bool Read(std::string * decompressed_data);

private:
  std::ifstream in_;
};  // class ProtoStreamReader

}  // namespace io

#endif  // OCC_GRIDMAP__PROTO_IO_HPP_
