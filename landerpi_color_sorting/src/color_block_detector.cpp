// Copyright 2026 ldavalle
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

#include "landerpi_color_sorting/color_block_detector.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace landerpi_color_sorting
{

ColorBlockDetector::ColorBlockDetector(ColorBlockDetectorConfig config)
: config_(std::move(config))
{
}

void ColorBlockDetector::SetTarget(const ColorTarget & target)
{
  for (auto & existing : targets_) {
    if (existing.name == target.name) {
      existing.lab = target.lab;
      return;
    }
  }
  targets_.push_back(target);
}

void ColorBlockDetector::ClearTargets()
{
  targets_.clear();
}

void ColorBlockDetector::SetTolerance(double l_tolerance, double ab_tolerance)
{
  config_.l_tolerance = l_tolerance;
  config_.ab_tolerance = ab_tolerance;
}

std::vector<BlockDetection> ColorBlockDetector::Detect(const cv::Mat & bgr_frame) const
{
  std::vector<BlockDetection> detections;
  if (bgr_frame.empty() || targets_.empty()) {
    return detections;
  }

  // Convert and blur once, shared by every color.
  cv::Mat lab;
  cv::cvtColor(bgr_frame, lab, cv::COLOR_BGR2Lab);
  cv::GaussianBlur(lab, lab, cv::Size(3, 3), 3);

  const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

  for (const auto & target : targets_) {
    const cv::Scalar lower(
      target.lab[0] - config_.l_tolerance,
      target.lab[1] - config_.ab_tolerance,
      target.lab[2] - config_.ab_tolerance);
    const cv::Scalar upper(
      target.lab[0] + config_.l_tolerance,
      target.lab[1] + config_.ab_tolerance,
      target.lab[2] + config_.ab_tolerance);

    cv::Mat mask;
    cv::inRange(lab, lower, upper, mask);
    cv::erode(mask, mask, kernel);
    cv::dilate(mask, mask, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto & contour : contours) {
      const double area = std::fabs(cv::contourArea(contour));
      if (area < config_.min_area_px || area > config_.max_area_px) {
        continue;
      }
      const cv::Moments m = cv::moments(contour);
      if (m.m00 <= 0.0) {
        continue;
      }
      BlockDetection detection;
      detection.color = target.name;
      detection.center_px = cv::Point2d(m.m10 / m.m00, m.m01 / m.m00);
      detection.area_px = area;
      detection.angle_deg = cv::minAreaRect(contour).angle;
      detection.bounding_box = cv::boundingRect(contour);
      const cv::Rect & box = detection.bounding_box;
      const int margin = config_.border_margin_px;
      detection.touches_image_border =
        box.x <= margin || box.y <= margin ||
        box.x + box.width >= bgr_frame.cols - margin ||
        box.y + box.height >= bgr_frame.rows - margin;
      detections.push_back(std::move(detection));
    }
  }

  std::sort(
    detections.begin(), detections.end(),
    [](const BlockDetection & a, const BlockDetection & b) {return a.area_px > b.area_px;});
  return detections;
}

std::optional<cv::Vec3d> ColorBlockDetector::SampleLab(
  const cv::Mat & bgr_frame, double x_frac, double y_frac, int patch_radius_px)
{
  if (bgr_frame.empty() || x_frac < 0.0 || x_frac > 1.0 || y_frac < 0.0 || y_frac > 1.0) {
    return std::nullopt;
  }
  const int cx = static_cast<int>(x_frac * (bgr_frame.cols - 1));
  const int cy = static_cast<int>(y_frac * (bgr_frame.rows - 1));
  const int side = 2 * patch_radius_px + 1;
  const cv::Rect patch = cv::Rect(cx - patch_radius_px, cy - patch_radius_px, side, side) &
    cv::Rect(0, 0, bgr_frame.cols, bgr_frame.rows);
  if (patch.area() <= 0) {
    return std::nullopt;
  }

  cv::Mat lab;
  cv::cvtColor(bgr_frame(patch), lab, cv::COLOR_BGR2Lab);
  const cv::Scalar mean = cv::mean(lab);
  return cv::Vec3d(mean[0], mean[1], mean[2]);
}

}  // namespace landerpi_color_sorting
