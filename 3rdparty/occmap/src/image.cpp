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
#include <utility>

#include "map2d/image.hpp"

namespace io
{

StreamFileWriter::StreamFileWriter(const std::string & filename)
: filename_(filename), out_(filename, std::ios::out | std::ios::binary) {}

StreamFileWriter::~StreamFileWriter() {}

bool StreamFileWriter::Write(const char * const data, const size_t len)
{
  if (out_.bad()) {
    return false;
  }
  out_.write(data, len);
  return !out_.bad();
}

bool StreamFileWriter::Close()
{
  if (out_.bad()) {
    return false;
  }
  out_.close();
  return !out_.bad();
}

bool StreamFileWriter::WriteHeader(const char * const data, const size_t len)
{
  if (out_.bad()) {
    return false;
  }
  out_.flush();
  out_.seekp(0);
  return Write(data, len);
}

std::string StreamFileWriter::GetFilename() {return filename_;}

Image::Image(int width, int height)
: width_(width), height_(height), pixels_(width * height, 0) {}

void Image::SetPixel(int x, int y, const uint8_t & color)
{
  pixels_[y * width_ + x] = color;
}

uint8_t Image::GetPixel(int x, int y) const
{
  return pixels_[y * width_ + x];
}
void Image::Rotate90DegreeClockWise()
{
  const auto old_pixels = pixels_;
  pixels_.clear();
  for (int x = 0; x < width_; ++x) {
    for (int y = height_ - 1; y >= 0; --y) {
      pixels_.push_back(old_pixels.at(y * width_ + x));
    }
  }
  std::swap(width_, height_);
}
void Image::Rotate90LeftDegree()
{
  const auto old_pixels = pixels_;
  pixels_.clear();
  for (int x = width_ - 1; x >= 0; --x) {
    for (int y = 0; y < height_; ++y) {
      pixels_.push_back(old_pixels.at(y * width_ + x));
    }
  }
  std::swap(width_, height_);
}

void Image::WritePgm(const double resolution, StreamFileWriter * const file_writer)
{
  const std::string header = "P5\n# Cartographer map; " +
    std::to_string(resolution) + " m/pixel\n" +
    std::to_string(width_) + " " +
    std::to_string(height_) + "\n255\n";
  file_writer->Write(header.data(), header.size());
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const char color = GetPixel(x, y);
      file_writer->Write(&color, 1);
    }
  }
}
}  // namespace io
