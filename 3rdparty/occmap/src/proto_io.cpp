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

#include <string>

#include "occ_gridmap/proto_io.hpp"

namespace
{

inline void FastGunzipString(
  const std::string & compressed,
  std::string * decompressed)
{
  boost::iostreams::filtering_ostream out;
  out.push(boost::iostreams::gzip_decompressor());
  out.push(boost::iostreams::back_inserter(*decompressed));
  boost::iostreams::write(
    out, reinterpret_cast<const char *>(compressed.data()),
    compressed.size());
}

inline
void FastGzipString(
  const std::string & uncompressed,
  std::string * compressed)
{
  boost::iostreams::filtering_ostream out;
  out.push(
    boost::iostreams::gzip_compressor(boost::iostreams::zlib::best_speed));
  out.push(boost::iostreams::back_inserter(*compressed));
  boost::iostreams::write(
    out,
    reinterpret_cast<const char *>(uncompressed.data()),
    uncompressed.size());
}

// First eight bytes to identify our proto stream format.
const uint64_t kMagic = 0x7b1d1f7b5bf501db;

void WriteSizeAsLittleEndian(uint64_t size, std::ostream * out)
{
  for (int i = 0; i != 8; ++i) {
    out->put(size & 0xff);
    size >>= 8;
  }
}

bool ReadSizeAsLittleEndian(std::istream * in, uint64_t * size)
{
  *size = 0;
  for (int i = 0; i != 8; ++i) {
    *size >>= 8;
    *size += static_cast<uint64_t>(in->get()) << 56;
  }
  return !in->fail();
}

}  // namespace

namespace io
{

ProtoStreamWriter::ProtoStreamWriter(const std::string & filename)
: out_(filename, std::ios::out | std::ios::binary)
{
  WriteSizeAsLittleEndian(kMagic, &out_);
}

void ProtoStreamWriter::Write(const std::string & uncompressed_data)
{
  std::string compressed_data;
  FastGzipString(uncompressed_data, &compressed_data);
  WriteSizeAsLittleEndian(compressed_data.size(), &out_);
  out_.write(compressed_data.data(), compressed_data.size());
}

void ProtoStreamWriter::WriteProto(const google::protobuf::Message & proto)
{
  std::string uncompressed_data;
  proto.SerializeToString(&uncompressed_data);
  Write(uncompressed_data);
}

bool ProtoStreamWriter::Close()
{
  out_.close();
  return !out_.fail();
}

ProtoStreamReader::ProtoStreamReader(const std::string & filename)
: in_(filename, std::ios::in | std::ios::binary)
{
  uint64_t magic;
  if (!ReadSizeAsLittleEndian(&in_, &magic) || magic != kMagic) {
    in_.setstate(std::ios::failbit);
  }
  if (!in_.good()) {std::cout << "Failed to open proto stream '" << filename << "'.\n";}
}

bool ProtoStreamReader::Read(std::string * decompressed_data)
{
  uint64_t compressed_size;
  if (!ReadSizeAsLittleEndian(&in_, &compressed_size)) {
    return false;
  }
  std::string compressed_data(compressed_size, '\0');
  if (!in_.read(&compressed_data.front(), compressed_size)) {
    return false;
  }
  FastGunzipString(compressed_data, decompressed_data);
  return true;
}

bool ProtoStreamReader::ReadProto(google::protobuf::Message * proto)
{
  std::string decompressed_data;
  return Read(&decompressed_data) && proto->ParseFromString(decompressed_data);
}

bool ProtoStreamReader::eof() const {return in_.eof();}

}  // namespace io
