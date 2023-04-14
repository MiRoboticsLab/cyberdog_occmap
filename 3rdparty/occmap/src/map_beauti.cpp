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

#include <vector>
#include <utility>
#include <queue>

#include "occ_gridmap/map_beauti.hpp"

namespace
{

constexpr float ringAreaThresh = 0.1f;
constexpr float contourThresh = 0.9f;


constexpr float freeProbability = 0.196f;
constexpr float occProbability = 0.65f;

constexpr uchar maxThresh = static_cast<uchar>((1 - freeProbability) * 255.f);
constexpr uchar minThresh = static_cast<uchar>((1 - occProbability) * 255.f);  // (1 - 0.65) * 255.f
constexpr uchar freeThresh = 128;
constexpr uchar occThresh = 1;
constexpr int noiseNumThresh = 1;

struct LidDis
{
  int lId;
  float distance;
};

struct NoiseArea
{
  int areaId = 0;
  int areaPixelNum = 0;
  bool containObstacle = false;
  std::vector<cv::Point> areaPixelCoord;
};

using LinePixel = std::vector<cv::Point>;
struct NormalPixel
{
  Eigen::Vector2d normal;
  LinePixel sidepixels;
  std::vector<LinePixel> linepixels;
};

bool IsOverFlow(const cv::Point & pixel, const int width, const int height)
{
  if (pixel.x < 0 || pixel.y < 0 || pixel.x > width || pixel.y > height) {return true;} else {
    return false;
  }
}

inline
std::pair<LidDis, cv::Vec4i> calDistanceLine(
  const cv::Point & pixel,
  const cv::Vec4i & line
)
{
  std::pair<LidDis, cv::Vec4i> disline;
  disline.first.distance = -1.f;
  Eigen::Vector2f p1(line[0], line[1]);
  Eigen::Vector2f p2(line[2], line[3]);
  Eigen::Vector2f p3(pixel.x, pixel.y);

  const float k = (p2.y() - p1.y()) / (p2.x() - p1.x());
  const float b = p2.y() - k * p2.x();
  const float x1 = (k * (p3.y() - b) + p3.x()) / (k * k + 1);
  const float y1 = k * x1 + b;
  Eigen::Vector2i p0(x1, y1);

  int dp1p0 = (p0 - p1.cast<int>()).norm();
  int dp2p0 = (p0 - p2.cast<int>()).norm();
  float dp1p2 = (p2.cast<int>() - p1.cast<int>()).norm();
  if (dp1p0 < dp1p2 && dp2p0 < dp1p2) {
    int dis = (p3.cast<int>() - p0).norm();
    disline.first.distance = dis < 10.f ? dis : -1.f;
    disline.second = line;
  }
  return disline;
}

inline
std::vector<cv::Point> BresenHam2D(
  const cv::Point & startpoint,
  const cv::Point & endpoint
)
{
  std::vector<cv::Point> linepoints;
  cv::Point dis = endpoint - startpoint;
  cv::Point abs_dis(std::abs(dis.x), std::abs(dis.y));
  cv::Point offset_dis;
  offset_dis.x = dis.x > 0 ? 1 : -1;
  offset_dis.y = dis.y > 0 ? 1 : -1;

  cv::Point temp_cell = startpoint;

  if (abs_dis.x >= abs_dis.y) {
    int error = -abs_dis.x;
    for (int ii = 0; ii < abs_dis.x; ++ii) {
      temp_cell.x += offset_dis.x;
      error += 2 * abs_dis.y;
      if (error > 0) {
        temp_cell.y += offset_dis.y;
        error -= 2 * abs_dis.x;
      }
      linepoints.emplace_back(temp_cell);
    }
  } else {
    int error = -abs_dis.y;
    for (int ii = 0; ii < abs_dis.y; ++ii) {
      temp_cell.y += offset_dis.y;
      error += 2 * abs_dis.x;
      if (error > 0) {
        temp_cell.x += offset_dis.x;
        error -= 2 * abs_dis.y;
      }
      linepoints.emplace_back(temp_cell);
    }
  }
  return linepoints;
}

inline
int IsInside(
  const cv::Point & pixel,
  const cv::Point & startpixel,
  Eigen::Vector2f & direction,
  const float & length,
  const cv::Mat & image
)
{
  int unkonw_num = 0, free_num = 0;
  bool unknow_flag = false, free_flag = false;
  int all_num = 0;
  direction.normalize();
  Eigen::Vector2f normal1(static_cast<float>(-direction.y()), static_cast<float>(direction.x()));
  cv::Point endpixel(startpixel.x + length * normal1.x(), startpixel.y + length * normal1.y());

  auto linepoints = BresenHam2D(startpixel, endpixel);
  for (const auto & linepixel : linepoints) {
    if (IsOverFlow(linepixel, image.size().width - 1, image.size().height - 1)) {continue;}
    const uchar & color = image.at<uchar>(linepixel.y, linepixel.x);
    if (color == 128) {unkonw_num++;}
    if (color > 205) {free_num++;}
    all_num++;
  }

  float unknow_scale = static_cast<float>(unkonw_num) / all_num;
  float free_scale = static_cast<float>(free_num) / all_num;
  if (unknow_scale > 0.7 && free_scale < 0.3) {
    unknow_flag = true;
  } else if (free_scale > 0.7 && unknow_scale < 0.3) {
    free_flag = true;
  }

  if (!unknow_flag && !free_flag) {return 0;}

  Eigen::Vector2f normal;
  if (unknow_flag) {normal = -normal1;}
  if (free_flag) {normal = normal1;}

  Eigen::Vector2f dir(pixel.x - startpixel.x, pixel.y - startpixel.y);
  const float cosangle = static_cast<float>(normal.dot(dir)) /
    static_cast<float>((normal.norm() * dir.norm()));
  if (cosangle < 0.f) {
    return -1;
  } else {
    return 1;
  }

  return 0;
}

}  // namespace

MapBeauti::MapBeauti(
  const MapBeautiParam & mapbeauti_param)
: mapbeauti_param_(mapbeauti_param) {}

MapBeauti::~MapBeauti() {}

std::vector<uint8_t> MapBeauti::process(
  const std::vector<uint8_t> & image, const int width,
  const int height)
{
  origin_image_ = image;
  cv::Size sz(width, height);
  cv::Mat probability_img(sz, CV_8UC1, (uchar*)image.data());
  if (probability_img.empty()) {
    std::cout << "[MapBeauti]: image is empty!" << std::endl;
    return origin_image_;
  }

  processNoisePixel(probability_img);

  cv::Mat flat = probability_img.reshape(1, probability_img.total() * probability_img.channels());
  std::vector<uint8_t> processed_img = probability_img.isContinuous() ? flat : flat.clone();
  return processed_img;
}

std::vector<uint8_t> MapBeauti::FinalProcess(
  const std::vector<uint8_t> & image, const int width,
  const int height)
{
  origin_image_ = image;
  cv::Size sz(width, height);
  cv::Mat probability_img(sz, CV_8UC1, (uchar*)image.data());
  if (probability_img.empty()) {
    std::cout << "[MapBeauti]: image is empty!" << std::endl;
    return origin_image_;
  }

  cv::Mat approx_img(probability_img.size().height,
    probability_img.size().width, CV_8UC1, cv::Scalar(128));
  if (!processImg(probability_img, approx_img)) {
    approx_img = probability_img;
  }
  cv::Mat flat = approx_img.reshape(1, approx_img.total() * approx_img.channels());
  std::vector<uint8_t> processed_img = approx_img.isContinuous() ? flat : flat.clone();
  return processed_img;
}

bool MapBeauti::processImg(const cv::Mat & probability_img, cv::Mat & approx_img)
{
  cv::Mat bin_img(probability_img.size().height, probability_img.size().width, CV_8UC1, cv::Scalar(
      0));
  cv::Mat insidebin_img(probability_img.size().height,
    probability_img.size().width, CV_8UC1, cv::Scalar(0));
  cv::Mat approxbin_img;
  bool internalopt_flag = false;
  int freegrid_num = 0;
  ProbabilityToBinImage(probability_img, bin_img, approx_img);
  approxbin_img = approx_img;
  ImageDilate(bin_img, bin_img, 3, freegrid_num);
  std::vector<std::vector<cv::Point>> approx_contours;
  if (!DetectPolyDP(bin_img, approx_contours)) {
    std::cout << "DetectPolyDP failed, display origin map" << std::endl;
    return false;
  }
  int contourgrid_num = 0;
  int insidepixel_num = 0;
  std::vector<cv::Point> approx_contour = approx_contours[0];
  if (approx_contour.size() != 0) {
    internalopt_flag = true;
    FillSideHoles(approx_contour, approx_img);
    FillInternal(
      approx_contour, probability_img, approx_img, insidebin_img, contourgrid_num,
      insidepixel_num);
    contour_scale_ = static_cast<float>(contourgrid_num) / static_cast<float>(freegrid_num);
    if (contour_scale_ < contourThresh) {
      std::cout << "[MapBeauti]: contour_scale_ below 0.9f, display origin map: " << std::to_string(
        contour_scale_) << std::endl;
      return false;
    }
    std::cout << "[MapBeauti]: contour_scale_ : " << std::to_string(contour_scale_) << std::endl;
  }

  cv::Scalar colors = cv::Scalar(0);
  cv::drawContours(approx_img, approx_contours, -1, colors, 1);

  float inside_scale = static_cast<float>(insidepixel_num) / static_cast<float>(contourgrid_num);
  if (internalopt_flag && inside_scale >= ringAreaThresh &&
    contour_scale_ > (ringAreaThresh + contourThresh))
  {
    processInside(probability_img, insidebin_img, approx_img, approxbin_img);
    std::cout << "[MapBeauti]: inside_scale : " << std::to_string(inside_scale) << std::endl;
  } else {
    std::cout << "[MapBeauti]: inside_scale below 0.1f, not optimation inside: " << std::to_string(
      inside_scale) << std::endl;
  }

  processNoisePixel(approx_img);

  return true;
}

cv::Mat MapBeauti::LineOptimation(const cv::Mat & probability_img)
{
  cv::Mat map_image = probability_img;
  std::vector<cv::Vec4i> lines = LineDetect(map_image);
  FitPixelToLine(lines, map_image);
  processNoisePixel(map_image);
  return map_image;
}

void MapBeauti::processInside(
  const cv::Mat & probability_img, const cv::Mat & insidebin_img,
  cv::Mat & approx_img, const cv::Mat & approxbin_img)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(insidebin_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.size() == 0) {return;}
  std::sort(
    contours.begin(), contours.end(),
    [&](std::vector<cv::Point> A, std::vector<cv::Point> B)
    {return A.size() < B.size();});
  std::vector<cv::Point> approx_contour;
  cv::approxPolyDP(
    contours[contours.size() - 1], approx_contour, mapbeauti_param_.approxPolythresh, true);
  std::cout << "[processInside]: approx_contour.size: " << approx_contour.size() << std::endl;
  if (approx_contour.size() < 5) {return;}
  std::vector<NormalPixel> normal_pixels;
  normal_pixels.reserve(approx_contour.size());
  for (size_t ii = 0; ii < approx_contour.size() - 1; ++ii) {
    Eigen::Vector2d direction(approx_contour[ii + 1].x - approx_contour[ii].x,
      approx_contour[ii + 1].y - approx_contour[ii].y);
    direction.normalize();
    Eigen::Vector2d normal(direction.y(), -direction.x());
    normal = -normal;
    NormalPixel normal_pixel;
    normal_pixel.normal = normal;
    normal_pixel.sidepixels = BresenHam2D(approx_contour[ii], approx_contour[ii + 1]);
    for (double ll = 0.f; ll < mapbeauti_param_.sideFillThresh; ll += 0.5f) {
      cv::Point startpixel, endpixel;
      startpixel.x = cvCeil(approx_contour[ii].x + ll * normal.x());
      startpixel.y = cvCeil(approx_contour[ii].y + ll * normal.y());
      endpixel.x = cvCeil(approx_contour[ii + 1].x + ll * normal.x());
      endpixel.y = cvCeil(approx_contour[ii + 1].y + ll * normal.y());
      std::vector<cv::Point> fillholepixel = BresenHam2D(startpixel, endpixel);
      normal_pixel.linepixels.emplace_back(fillholepixel);
    }
    normal_pixels.emplace_back(normal_pixel);
  }

  for (const auto & normal_pixel : normal_pixels) {
    for (const auto & pixel : normal_pixel.sidepixels) {
      uchar & color = approx_img.at<uchar>(pixel.y, pixel.x);
      color = 0;
      cv::Point endpixel;
      endpixel.x = cvRound(pixel.x + 6.f * normal_pixel.normal.x());
      endpixel.y = cvRound(pixel.y + 6.f * normal_pixel.normal.y());
      std::vector<cv::Point> fillholepixels = BresenHam2D(pixel, endpixel);
      for (const auto & fillholepixel : fillholepixels) {
        if (IsOverFlow(fillholepixel, approx_img.size().width - 1, approx_img.size().height - 1)) {
          continue;
        }
        uchar & approx_color = approx_img.at<uchar>(fillholepixel.y, fillholepixel.x);
        approx_color = 255;
      }
    }
    for (const auto & linepixel : normal_pixel.linepixels) {
      for (const auto & pixel : linepixel) {
        if (IsOverFlow(pixel, approx_img.size().width - 1, approx_img.size().height - 1)) {
          continue;
        }
        uchar & color = approx_img.at<uchar>(pixel.y, pixel.x);
        color = 255;
      }
    }
  }
  for (int row = approx_img.rows - 1; row >= 0; --row) {
    for (int col = 0; col < approx_img.cols; ++col) {
      cv::Point pixel(col, row);
      double distance = cv::pointPolygonTest(approx_contour, pixel, false);
      uchar & color = approx_img.at<uchar>(row, col);
      if (distance > 0.f) {
        color = 128;
      }
      if (distance < 0.f && color == 128) {
        color = probability_img.at<uchar>(row, col);
      }
    }
  }
  std::vector<std::vector<cv::Point>> approx_contours;
  approx_contours.emplace_back(approx_contour);
  cv::Scalar colors = cv::Scalar(0);
  cv::drawContours(approx_img, approx_contours, -1, colors, 1);
}

void MapBeauti::ProbabilityToBinImage(
  const cv::Mat & probability_img,
  cv::Mat & bin_img,
  cv::Mat & approx_img)
{
  for (int row = probability_img.rows - 1; row >= 0; --row) {
    const uchar * img_row = probability_img.ptr(row);
    for (int col = 0; col < probability_img.cols; ++col) {
      uchar probability_color = img_row[col];
      uchar & color = bin_img.at<uchar>(row, col);
      uchar & approx_color = approx_img.at<uchar>(row, col);
      if (probability_color == freeThresh || probability_color < minThresh) {
        color = 0;
      } else {
        color = 255;
      }
      if (probability_color >= maxThresh) {
        approx_color = 255;
      }
    }
  }  // for
}  // ProbabilityToBinImage

void MapBeauti::ImageDilate(
  cv::Mat & input, cv::Mat & output, const int & kernelSize,
  int & freegrid_num)
{
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
  cv::dilate(input, output, kernel);

  for (int row = output.rows - 1; row >= 0; --row) {
    const uchar * img_row = output.ptr(row);
    for (int col = 0; col < output.cols; ++col) {
      double probability = 1.f - static_cast<double>(img_row[col]) / 255.f;
      if (probability <= 0.196f) {
        freegrid_num++;
      }
    }
  }  // for
}

bool MapBeauti::DetectPolyDP(
  const cv::Mat & bin_img,
  std::vector<std::vector<cv::Point>> & approx_contours)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bin_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  std::sort(
    contours.begin(), contours.end(),
    [&](std::vector<cv::Point> A, std::vector<cv::Point> B)
    {return A.size() < B.size();});
  std::vector<cv::Point> approx_contour;
  if (contours.size() == 0) {return false;}
  cv::approxPolyDP(
    contours[contours.size() - 1], approx_contour, mapbeauti_param_.approxPolythresh, true);
  approx_contours.emplace_back(approx_contour);
  if (approx_contour.size() < 5) {return false;}
  return true;
}

void MapBeauti::FillSideHoles(const std::vector<cv::Point> & approx_contour, cv::Mat & approx_img)
{
  std::vector<NormalPixel> normal_pixels;
  normal_pixels.reserve(approx_contour.size());
  for (size_t ii = 0; ii < approx_contour.size() - 1; ++ii) {
    Eigen::Vector2d direction(approx_contour[ii + 1].x - approx_contour[ii].x,
      approx_contour[ii + 1].y - approx_contour[ii].y);
    direction.normalize();
    Eigen::Vector2d normal(direction.y(), -direction.x());
    NormalPixel normal_pixel;
    normal_pixel.normal = normal;
    normal_pixel.sidepixels = BresenHam2D(approx_contour[ii], approx_contour[ii + 1]);
    for (double ll = 0.f; ll < mapbeauti_param_.sideFillThresh; ll += 0.5f) {
      cv::Point startpixel, endpixel;
      startpixel.x = cvCeil(approx_contour[ii].x + ll * normal.x());
      startpixel.y = cvCeil(approx_contour[ii].y + ll * normal.y());
      endpixel.x = cvCeil(approx_contour[ii + 1].x + ll * normal.x());
      endpixel.y = cvCeil(approx_contour[ii + 1].y + ll * normal.y());
      if (IsOverFlow(startpixel, approx_img.size().width - 1, approx_img.size().height - 1) ||
        IsOverFlow(endpixel, approx_img.size().width - 1, approx_img.size().height - 1))
      {
        continue;
      }
      std::vector<cv::Point> fillholepixel = BresenHam2D(startpixel, endpixel);
      normal_pixel.linepixels.emplace_back(fillholepixel);
    }
    normal_pixels.emplace_back(normal_pixel);
  }
  for (const auto & normal_pixel : normal_pixels) {
    for (const auto & pixel : normal_pixel.sidepixels) {
      if (IsOverFlow(pixel, approx_img.size().width - 1, approx_img.size().height - 1)) {
        continue;
      }
      uchar & color = approx_img.at<uchar>(pixel.y, pixel.x);
      color = 0;
      cv::Point endpixel;
      endpixel.x = cvRound(pixel.x + 6.f * normal_pixel.normal.x());
      endpixel.y = cvRound(pixel.y + 6.f * normal_pixel.normal.y());
      if (IsOverFlow(endpixel, approx_img.size().width - 1, approx_img.size().height - 1)) {
        continue;
      }
      std::vector<cv::Point> fillholepixels = BresenHam2D(pixel, endpixel);
      for (const auto & fillholepixel : fillholepixels) {
        if (IsOverFlow(fillholepixel, approx_img.size().width - 1, approx_img.size().height - 1)) {
          continue;
        }
        uchar & approx_color = approx_img.at<uchar>(fillholepixel.y, fillholepixel.x);
        if (approx_color == 0) {break;}
        if (approx_color == 128) {
          approx_color = 255;
        }
      }
    }
    for (const auto & linepixel : normal_pixel.linepixels) {
      for (const auto & pixel : linepixel) {
        if (IsOverFlow(pixel, approx_img.size().width - 1, approx_img.size().height - 1)) {
          continue;
        }
        uchar & color = approx_img.at<uchar>(pixel.y, pixel.x);
        color = 255;
      }
    }
  }
}

void MapBeauti::FillInternal(
  const std::vector<cv::Point> & approx_contour,
  const cv::Mat & probability_img,
  cv::Mat & approx_img,
  cv::Mat & insidebin_img,
  int & contourpixel_num,
  int & insidepixel_num
)
{
  for (int row = approx_img.rows - 1; row >= 0; --row) {
    for (int col = 0; col < approx_img.cols; ++col) {
      cv::Point pixel(col, row);
      double distance = cv::pointPolygonTest(approx_contour, pixel, false);
      uchar & color = approx_img.at<uchar>(pixel.y, pixel.x);
      uchar & inside_color = insidebin_img.at<uchar>(pixel.y, pixel.x);
      if (distance > 0.f && color == 128) {
        color = probability_img.at<uchar>(pixel.y, pixel.x);
      } else if (distance < 0.f) {
        color = 128;
      }
      if (distance > 0.f) {contourpixel_num++;}
      if (distance > 0.f && color == 128) {
        inside_color = 255;
        insidepixel_num++;
      } else {
        inside_color = 0;
      }
    }
  }
}

std::vector<cv::Vec4i> MapBeauti::LineDetect(const cv::Mat & map_image)
{
  cv::Mat canny_img(map_image.size().height, map_image.size().width, CV_8UC1, cv::Scalar(0));
  for (int row = map_image.rows - 1; row >= 0; --row) {
    const uchar * img_row = map_image.ptr(row);
    for (int col = 0; col < map_image.cols; ++col) {
      double probability = 1.f - static_cast<double>(img_row[col]) / 255.f;
      uchar & color = canny_img.at<uchar>(row, col);
      if (probability >= 0.65f) {
        color = 255;
      } else if (probability <= 0.196f) {
        color = 0;
      } else {
        color = 0;
      }
    }
  }  // for
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(canny_img, lines, 1, CV_PI / 180, 70, 5, 20);
  return lines;
}

void MapBeauti::FitPixelToLine(const std::vector<cv::Vec4i> & lines, cv::Mat & map_image)
{
  for (int row = map_image.rows - 1; row >= 0; --row) {
    uchar * img_row = map_image.ptr(row);
    for (int col = 0; col < map_image.cols; ++col) {
      double probability = 1.f - static_cast<double>(img_row[col]) / 255.f;
      cv::Point pixel(col, row);
      if (probability >= 0.196f) {
        std::vector<std::pair<LidDis, cv::Vec4i>> dislines;
        for (size_t lId = 0; lId < lines.size(); lId++) {
          auto disline = calDistanceLine(pixel, lines[lId]);
          if (disline.first.distance != -1.f) {
            disline.first.lId = lId;
            dislines.emplace_back(disline);
          }
        }
        std::sort(
          dislines.begin(), dislines.end(),
          [&](std::pair<LidDis, cv::Vec4i> A, std::pair<LidDis, cv::Vec4i> B)
          {return A.first.distance < B.first.distance;});
        if (dislines.size() != 0) {
          uchar & color = img_row[col];
          cv::Point startpixel(dislines[0].second[0], dislines[0].second[1]);
          cv::Point endpixel(dislines[0].second[2], dislines[0].second[3]);
          cv::Point middlepixel = (startpixel + endpixel) / 2;
          Eigen::Vector2f direction(endpixel.x - startpixel.x, endpixel.y - startpixel.y);
          int status = IsInside(pixel, middlepixel, direction, 10.f, map_image);
          if (status == 1) {
            color = 255;
          } else {
            color = 128;
          }
        }
      } else {
        uchar & color = img_row[col];
        color = 255;
      }
    }
  }  // for
  for (size_t i = 0; i < lines.size(); i++) {
    cv::line(
      map_image, cv::Point(lines[i][0], lines[i][1]),
      cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0), 1);
  }
}

void MapBeauti::processNoisePixel(cv::Mat & map_image)
{
  std::vector<NoiseArea> noise_areas;
  cv::Mat flag_img(map_image.size().height, map_image.size().width, CV_8UC1, cv::Scalar(0));
  for (int row = 0; row < map_image.rows; ++row) {
    for (int col = 0; col < map_image.cols; ++col) {
      cv::Point pixel(col, row);
      if (IsOverFlow(pixel, map_image.cols - 1, map_image.rows - 1)) {continue;}
      uchar & color = map_image.at<uchar>(row, col);
      uchar & flag = flag_img.at<uchar>(row, col);
      if (flag != occThresh && color <= minThresh && color != freeThresh) {
        NoiseArea noise_area;
        noise_area.areaId = static_cast<int>(noise_areas.size());
        noise_area.areaPixelNum = 1;
        noise_area.areaPixelCoord.emplace_back(pixel);
        flag = occThresh;
        std::queue<std::pair<int, int>> neighbors;
        neighbors.push({row, col});
        while (!neighbors.empty()) {
          auto rc = neighbors.front();
          neighbors.pop();
          int r = rc.first, c = rc.second;
          constexpr int siderows[8] = {0, -1, -1, -1, 0, 1, 1, 1};
          constexpr int sidecols[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
          for (int i = 0; i < 8; ++i) {
            int nextrow = r + siderows[i];
            int nextcol = c + sidecols[i];
            cv::Point nextPixel(nextcol, nextrow);
            if (IsOverFlow(nextPixel, map_image.cols - 1, map_image.rows - 1)) {continue;}
            uchar & nextcolor = map_image.at<uchar>(nextrow, nextcol);
            uchar & nextflag = flag_img.at<uchar>(nextrow, nextcol);
            if (nextflag != occThresh && nextcolor <= minThresh && nextcolor != freeThresh) {
              nextflag = occThresh;
              neighbors.push({nextrow, nextcol});
              noise_area.areaPixelNum += 1;
              noise_area.areaPixelCoord.emplace_back(nextPixel);
            }  // if
          }  // for
        }  // while
        noise_areas.emplace_back(noise_area);
      }  // if
      if (color > minThresh && color != freeThresh) {color = 255;}
    }  // for
  }  // for

  for (const auto & noise_area : noise_areas) {
    if (noise_area.areaPixelNum <= noiseNumThresh) {
      for (const auto & pixel : noise_area.areaPixelCoord) {
        uchar & color = map_image.at<uchar>(pixel.y, pixel.x);
        color = 255;
      }
    }
  }
}
