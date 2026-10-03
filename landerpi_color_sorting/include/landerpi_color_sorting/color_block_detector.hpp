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

#pragma once

#include <optional>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace landerpi_color_sorting
{

// One calibrated color: its name (e.g. "red") and its reference value in Lab
// space as returned by cv::cvtColor with COLOR_BGR2Lab (L in [0, 255], a/b in
// [0, 255], 128 = neutral).
struct ColorTarget
{
  std::string name;
  cv::Vec3d lab{0.0, 0.0, 0.0};
};

struct BlockDetection
{
  std::string color;
  // Contour centroid, in pixels.
  cv::Point2d center_px;
  double area_px = 0.0;
  // minAreaRect angle, degrees. Lets the gripper line up with the block's
  // faces in later stages.
  double angle_deg = 0.0;
  cv::Rect bounding_box;
  // The block is cut by the image edge: its visible centroid is not the real
  // block center, so it must not be picked from this frame.
  bool touches_image_border = false;
};

struct ColorBlockDetectorConfig
{
  // Lightness changes a lot with shadows/lighting, chroma (a/b) much less,
  // hence the looser L tolerance — same criterion as B2's LineTracker.
  double l_tolerance = 60.0;
  double ab_tolerance = 20.0;
  // A ~3 cm cube at the observation distance covers a few thousand px at
  // 640x400; these bounds reject sensor noise and big same-hued regions.
  double min_area_px = 300.0;
  double max_area_px = 30000.0;
  // A bounding box within this many px of any image edge counts as cut.
  int border_margin_px = 2;
};

// Pure OpenCV multi-color block detector: no rclcpp dependency, so it can be
// gtest-ed directly on saved frames without bringing up a ROS node.
//
// Derived from B2's LineTracker (Lab + inRange + erode/dilate + contours),
// generalized from one color in fixed bands to N colors over the whole frame,
// reporting every block found instead of a single line position.
class ColorBlockDetector
{
public:
  explicit ColorBlockDetector(ColorBlockDetectorConfig config);

  // All blocks of every calibrated color, largest first. Empty if no color is
  // calibrated yet.
  std::vector<BlockDetection> Detect(const cv::Mat & bgr_frame) const;

  // Adds the color, or replaces it if one with the same name already exists.
  void SetTarget(const ColorTarget & target);
  void ClearTargets();
  const std::vector<ColorTarget> & targets() const {return targets_;}

  void SetTolerance(double l_tolerance, double ab_tolerance);

  // Mean Lab value of a square patch centered at (x_frac, y_frac), both in
  // [0, 1] of the image size. Used by the click calibration flow. nullopt if
  // the point falls outside the image.
  static std::optional<cv::Vec3d> SampleLab(
    const cv::Mat & bgr_frame, double x_frac, double y_frac, int patch_radius_px = 8);

private:
  ColorBlockDetectorConfig config_;
  std::vector<ColorTarget> targets_;
};

}  // namespace landerpi_color_sorting
