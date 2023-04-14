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

#ifndef MAP2D__IMAGE_HPP_
#define MAP2D__IMAGE_HPP_

#include <glog/logging.h>

#include <memory>
#include <fstream>
#include <functional>
#include <array>
#include <cstdint>
#include <vector>
#include <string>

namespace io
{

// An Implementation of file using std::ofstream.
class StreamFileWriter
{
public:
  ~StreamFileWriter();

  explicit StreamFileWriter(const std::string & filename);

  bool Write(const char * data, size_t len);
  bool WriteHeader(const char * data, size_t len);
  bool Close();
  std::string GetFilename();

private:
  const std::string filename_;
  std::ofstream out_;
};

class Image
{
public:
  Image(int width, int height);

  uint8_t GetPixel(int x, int y) const;

  const std::vector<uint8_t> & pixel_data() const {return pixels_;}

  void SetPixel(int x, int y, const uint8_t & color);

  void Rotate90DegreeClockWise();

  void Rotate90LeftDegree();

  int width() const {return width_;}

  int height() const {return height_;}

  void WritePgm(const double resolution, StreamFileWriter * const file_writer);

  void SetPixelData(const std::vector<uint8_t> & pixels_data) {pixels_ = pixels_data;}

private:
  int width_;
  int height_;
  std::vector<uint8_t> pixels_;
};  // class Image


}  // namespace io
#endif  // MAP2D__IMAGE_HPP_
