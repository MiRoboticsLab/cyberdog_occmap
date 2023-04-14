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

#ifndef OCC_GRIDMAP__COMMON_HPP_
#define OCC_GRIDMAP__COMMON_HPP_

#include <Eigen/Core>
#include <math.h>
#include <dirent.h>
#include <glog/logging.h>

#include <boost/filesystem.hpp>

#include <chrono>
#include <ratio>
#include <vector>
#include <fstream>
#include <string>

namespace common
{

class TicToc
{
public:
  TicToc()
  {
    tic();
  }

  void tic()
  {
    start_ = std::chrono::steady_clock::now();
  }

  double toc()
  {
    end_ = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end_ - start_).count();  // ms
  }

private:
  std::chrono::time_point<std::chrono::steady_clock> start_, end_;
};  // class TicToc

inline int RoundToInt(const double x) {return std::lround(x);}

inline int RoundToInt(const float x) {return std::lround(x);}

inline int64_t RoundToInt64(const double x) {return std::lround(x);}

inline int64_t RoundToInt64(const float x) {return std::lround(x);}

inline bool createFolder(const std::string & path)
{
  return boost::filesystem::create_directories(path);
}

inline void GetFileNames(
  std::vector<std::string> & filenames,
  const std::string & path)
{
  struct dirent * entry;
  DIR * dir = opendir(path.c_str());
  if (dir == NULL) {
    return;
  }
  while ((entry = readdir(dir)) != NULL) {
    std::string str(entry->d_name);
    if (str.size() < 8) {continue;}
    filenames.emplace_back(str);
  }
  closedir(dir);
}

template<typename T>
T Clamp(const T value, const T min, const T max)
{
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

inline
void WritePointCloudTxt(
  const Eigen::Vector2d & point,
  const Eigen::Vector2d & origin,
  const std::string & file_path)
{
  std::ofstream outfile(file_path.data(), std::ios::app);
  outfile << point.x() << " " <<
    point.y() << " " <<
    origin.x() << " " <<
    origin.y() << "\n";
  outfile.close();
}

constexpr int64_t kUtsEpochOffsetFromUnixEpochInSeconds =
  (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock
{
  using rep = int64_t;
  using period = std::ratio<1, 10000000>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;

static inline Duration FromSeconds(double seconds)
{
  return std::chrono::duration_cast<Duration>(
    std::chrono::duration<double>(seconds));
}

static inline Time FromUniversal(int64_t ticks)
{
  return Time(Duration(ticks));
}

static inline int64_t ToUniversal(Time time)
{
  return time.time_since_epoch().count();
}

}  // namespace common
#endif  // OCC_GRIDMAP__COMMON_HPP_
