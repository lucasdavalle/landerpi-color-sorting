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

#include <gtest/gtest.h>

#include <map>
#include <string>

#include "landerpi_color_sorting/color_block_detector.hpp"

using landerpi_color_sorting::BlockDetection;
using landerpi_color_sorting::ColorBlockDetector;
using landerpi_color_sorting::ColorBlockDetectorConfig;
using landerpi_color_sorting::ColorTarget;

namespace
{

// Real 640x400 frame from the robot's camera (2026-10-03): the three Hiwonder
// cubes in a row on the light wooden test table, blue / red / green from left
// to right, with the arm's shadow at the bottom edge.
const char kThreeBlocksRow[] = TEST_DATA_DIR "/three_blocks_row.png";
// Same table and lighting, blocks scattered: red upright, green rotated
// ~20 deg and partly in shadow, blue rotated and cut by the bottom edge and
// the arm's shadow.
const char kThreeBlocksScattered[] = TEST_DATA_DIR "/three_blocks_scattered.png";
// Red and green side by side with their faces touching, blue apart.
const char kRedGreenTouching[] = TEST_DATA_DIR "/red_green_touching.png";
// Red and green fully visible, blue about half out past the left edge.
const char kBlueCutByLeftEdge[] = TEST_DATA_DIR "/blue_cut_by_left_edge.png";

// Block centers measured by hand on that frame.
const std::map<std::string, cv::Point2d> kExpectedCenters = {
  {"blue", {193.0, 307.0}},
  {"red", {289.0, 307.0}},
  {"green", {378.0, 307.0}},
};

// Calibrates the detector the same way the click flow will: sampling the
// frame itself at a point on each block.
ColorBlockDetector CalibratedOn(const cv::Mat & frame)
{
  ColorBlockDetector detector{ColorBlockDetectorConfig{}};
  for (const auto & [name, center] : kExpectedCenters) {
    const auto lab = ColorBlockDetector::SampleLab(
      frame, center.x / (frame.cols - 1), center.y / (frame.rows - 1));
    EXPECT_TRUE(lab.has_value());
    detector.SetTarget(ColorTarget{name, *lab});
  }
  return detector;
}

cv::Mat SolidFrame(const cv::Scalar & bgr)
{
  return cv::Mat(400, 640, CV_8UC3, bgr);
}

}  // namespace

TEST(ColorBlockDetector, FindsTheThreeRealBlocksAndNothingElse)
{
  const cv::Mat frame = cv::imread(kThreeBlocksRow);
  ASSERT_FALSE(frame.empty()) << "missing test image " << kThreeBlocksRow;

  const auto detections = CalibratedOn(frame).Detect(frame);

  ASSERT_EQ(detections.size(), 3u);
  for (const BlockDetection & d : detections) {
    ASSERT_EQ(kExpectedCenters.count(d.color), 1u) << d.color;
    const cv::Point2d expected = kExpectedCenters.at(d.color);
    EXPECT_NEAR(d.center_px.x, expected.x, 15.0) << d.color;
    EXPECT_NEAR(d.center_px.y, expected.y, 15.0) << d.color;
    EXPECT_GT(d.area_px, 1000.0) << d.color;
    EXPECT_FALSE(d.touches_image_border) << d.color;
  }
}

// Real use: calibrate once, then the blocks move. Calibration taken on the
// row frame must still find every block in the scattered one.
TEST(ColorBlockDetector, CalibrationOnOneFrameFindsBlocksInAnother)
{
  const cv::Mat calibration_frame = cv::imread(kThreeBlocksRow);
  const cv::Mat frame = cv::imread(kThreeBlocksScattered);
  ASSERT_FALSE(calibration_frame.empty());
  ASSERT_FALSE(frame.empty()) << "missing test image " << kThreeBlocksScattered;

  const auto detections = CalibratedOn(calibration_frame).Detect(frame);

  // Top-face centers measured by hand on the scattered frame. Tolerance is
  // looser than in the row test: green's side face partly merges with its top
  // face at this viewing angle, pulling its centroid sideways. Blue is cut by
  // the bottom edge and green's side face reaches it too: both get flagged.
  const std::map<std::string, cv::Point2d> expected = {
    {"red", {290.0, 256.0}},
    {"blue", {208.0, 365.0}},
    {"green", {388.0, 355.0}},
  };
  ASSERT_EQ(detections.size(), 3u);
  for (const BlockDetection & d : detections) {
    ASSERT_EQ(expected.count(d.color), 1u) << d.color;
    EXPECT_NEAR(d.center_px.x, expected.at(d.color).x, 20.0) << d.color;
    EXPECT_NEAR(d.center_px.y, expected.at(d.color).y, 20.0) << d.color;
    EXPECT_EQ(d.touches_image_border, d.color != "red") << d.color;
  }
}

// Touching blocks of different colors must come out as two detections, each
// centered on its own top face, not merged into one.
TEST(ColorBlockDetector, TouchingBlocksOfDifferentColorsStaySeparate)
{
  const cv::Mat calibration_frame = cv::imread(kThreeBlocksRow);
  const cv::Mat frame = cv::imread(kRedGreenTouching);
  ASSERT_FALSE(calibration_frame.empty());
  ASSERT_FALSE(frame.empty()) << "missing test image " << kRedGreenTouching;

  const auto detections = CalibratedOn(calibration_frame).Detect(frame);

  // Top-face centers measured by hand.
  const std::map<std::string, cv::Point2d> expected = {
    {"red", {304.0, 315.0}},
    {"green", {361.0, 310.0}},
    {"blue", {198.0, 251.0}},
  };
  ASSERT_EQ(detections.size(), 3u);
  for (const BlockDetection & d : detections) {
    ASSERT_EQ(expected.count(d.color), 1u) << d.color;
    EXPECT_NEAR(d.center_px.x, expected.at(d.color).x, 15.0) << d.color;
    EXPECT_NEAR(d.center_px.y, expected.at(d.color).y, 15.0) << d.color;
  }
}

// A block cut by the image edge is still reported (the node must know
// something is there) but flagged: its visible centroid is off from the real
// block center, so picking from this frame would miss.
TEST(ColorBlockDetector, BlockCutByTheImageEdgeIsFlagged)
{
  const cv::Mat calibration_frame = cv::imread(kThreeBlocksRow);
  const cv::Mat frame = cv::imread(kBlueCutByLeftEdge);
  ASSERT_FALSE(calibration_frame.empty());
  ASSERT_FALSE(frame.empty()) << "missing test image " << kBlueCutByLeftEdge;

  const auto detections = CalibratedOn(calibration_frame).Detect(frame);

  ASSERT_EQ(detections.size(), 3u);
  for (const BlockDetection & d : detections) {
    EXPECT_EQ(d.touches_image_border, d.color == "blue") << d.color;
  }
}

TEST(ColorBlockDetector, OneCalibratedColorOnlyFindsThatBlock)
{
  const cv::Mat frame = cv::imread(kThreeBlocksRow);
  ASSERT_FALSE(frame.empty());

  ColorBlockDetector detector{ColorBlockDetectorConfig{}};
  const auto lab = ColorBlockDetector::SampleLab(
    frame, kExpectedCenters.at("red").x / 639.0, kExpectedCenters.at("red").y / 399.0);
  ASSERT_TRUE(lab.has_value());
  detector.SetTarget(ColorTarget{"red", *lab});

  const auto detections = detector.Detect(frame);
  ASSERT_EQ(detections.size(), 1u);
  EXPECT_EQ(detections[0].color, "red");
}

TEST(ColorBlockDetector, NoTargetsMeansNoDetections)
{
  const cv::Mat frame = cv::imread(kThreeBlocksRow);
  ASSERT_FALSE(frame.empty());

  ColorBlockDetector detector = CalibratedOn(frame);
  detector.ClearTargets();
  EXPECT_TRUE(detector.Detect(frame).empty());
}

TEST(ColorBlockDetector, SetTargetReplacesSameName)
{
  ColorBlockDetector detector{ColorBlockDetectorConfig{}};
  detector.SetTarget(ColorTarget{"red", {90.0, 187.0, 156.0}});
  detector.SetTarget(ColorTarget{"red", {80.0, 180.0, 150.0}});
  ASSERT_EQ(detector.targets().size(), 1u);
  EXPECT_DOUBLE_EQ(detector.targets()[0].lab[0], 80.0);
}

TEST(ColorBlockDetector, IgnoresBlobsBelowMinArea)
{
  cv::Mat frame = SolidFrame(cv::Scalar(200, 200, 200));
  // 10x10 px red dot (area ~100 < default 300) and a 60x60 red block.
  cv::rectangle(frame, cv::Rect(50, 50, 10, 10), cv::Scalar(0, 0, 220), cv::FILLED);
  cv::rectangle(frame, cv::Rect(300, 200, 60, 60), cv::Scalar(0, 0, 220), cv::FILLED);

  ColorBlockDetector detector{ColorBlockDetectorConfig{}};
  const auto lab = ColorBlockDetector::SampleLab(frame, 330.0 / 639.0, 230.0 / 399.0);
  ASSERT_TRUE(lab.has_value());
  detector.SetTarget(ColorTarget{"red", *lab});

  const auto detections = detector.Detect(frame);
  ASSERT_EQ(detections.size(), 1u);
  EXPECT_NEAR(detections[0].center_px.x, 329.5, 2.0);
  EXPECT_NEAR(detections[0].center_px.y, 229.5, 2.0);
}

TEST(ColorBlockDetector, ReportsEveryBlockOfTheSameColorLargestFirst)
{
  cv::Mat frame = SolidFrame(cv::Scalar(200, 200, 200));
  cv::rectangle(frame, cv::Rect(100, 100, 40, 40), cv::Scalar(0, 180, 0), cv::FILLED);
  cv::rectangle(frame, cv::Rect(400, 200, 70, 70), cv::Scalar(0, 180, 0), cv::FILLED);

  ColorBlockDetector detector{ColorBlockDetectorConfig{}};
  const auto lab = ColorBlockDetector::SampleLab(frame, 120.0 / 639.0, 120.0 / 399.0);
  ASSERT_TRUE(lab.has_value());
  detector.SetTarget(ColorTarget{"green", *lab});

  const auto detections = detector.Detect(frame);
  ASSERT_EQ(detections.size(), 2u);
  EXPECT_GT(detections[0].area_px, detections[1].area_px);
  EXPECT_NEAR(detections[0].center_px.x, 434.5, 2.0);
}

TEST(ColorBlockDetector, SampleLabRejectsPointsOutsideTheImage)
{
  const cv::Mat frame = SolidFrame(cv::Scalar(200, 200, 200));
  EXPECT_FALSE(ColorBlockDetector::SampleLab(frame, -0.1, 0.5).has_value());
  EXPECT_FALSE(ColorBlockDetector::SampleLab(frame, 0.5, 1.1).has_value());
  EXPECT_FALSE(ColorBlockDetector::SampleLab(cv::Mat(), 0.5, 0.5).has_value());
  EXPECT_TRUE(ColorBlockDetector::SampleLab(frame, 1.0, 1.0).has_value());
}
