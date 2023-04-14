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

#ifndef OCC_GRIDMAP__MAP_BEAUTI_HPP_
#define OCC_GRIDMAP__MAP_BEAUTI_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

#include <vector>

#include "occ_gridmap/param.hpp"

class MapBeauti
{
public:
  explicit MapBeauti(const MapBeautiParam & mapbeauti_param);
  ~MapBeauti();

  std::vector<uint8_t> process(
    const std::vector<uint8_t> & image, const int width,
    const int height);
  std::vector<uint8_t> FinalProcess(
    const std::vector<uint8_t> & image, const int width,
    const int height);

private:
  bool processImg(const cv::Mat & probability_img, cv::Mat & approx_img);

  cv::Mat LineOptimation(const cv::Mat & probability_img);

  // poly optimation

  void ProbabilityToBinImage(
    const cv::Mat & probability_img, cv::Mat & bin_img,
    cv::Mat & approx_img);

  void ImageDilate(cv::Mat & input, cv::Mat & output, const int & kernelSize, int & freegrid_num);

  bool DetectPolyDP(const cv::Mat & bin_img, std::vector<std::vector<cv::Point>> & approx_contours);

  void FillSideHoles(const std::vector<cv::Point> & approx_contour, cv::Mat & approx_img);

  void FillInternal(
    const std::vector<cv::Point> & approx_contour, const cv::Mat & probability_img,
    cv::Mat & approx_img, cv::Mat & insidebin_img, int & contourpixel_num,
    int & insidepixel_num);

  // line optimation

  std::vector<cv::Vec4i> LineDetect(const cv::Mat & map_image);

  void FitPixelToLine(const std::vector<cv::Vec4i> & lines, cv::Mat & image);

  // processNoise

  void processNoisePixel(cv::Mat & map_image);

  // inside

  void processInside(
    const cv::Mat & probability_img, const cv::Mat & insidebin_img,
    cv::Mat & approx_img, const cv::Mat & approxbin_img);

private:
  MapBeautiParam mapbeauti_param_;
  size_t last_PolyDPSize_ = 0;
  std::vector<uint8_t> origin_image_;
  float contour_scale_ = 0.f;
};  // class MapBeauti

#endif  // OCC_GRIDMAP__MAP_BEAUTI_HPP_
