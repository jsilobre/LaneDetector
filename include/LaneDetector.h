#pragma once

#include <opencv2/core.hpp>

struct Line {
  Line() : p1(0, 0), p2(0, 0), norm{ 0 }{}
  Line(cv::Point p1_, cv::Point  p2_) : p1{ p1_ }, p2{ p2_ } {
    norm = cv::norm(p1 - p2); 
  }
  cv::Point p1, p2;
  double norm;
};

class LaneDetector {
public:
  LaneDetector() = default;
  ~LaneDetector() = default;

  void processFrame(const cv::Mat& input_frame);

private:
  /// Computes the mask for the ROI zone corresponding to the road (cut sky and borders)
  void computeMask(const cv::Size size) noexcept;
  /// Computes the horizon and car lines
  void computeBasicLines(const cv::Mat & frame) noexcept;
  /// Calculates the intersections point of two lines.
  /// Return true if the lines intersects, false otherwise
  bool computeIntersection(const Line& line1, const Line& line2, cv::Point &intersection_point) const noexcept;
  /// Computes the yellow and white masks for line detections
  void colorMask(cv::Mat& frame, cv::Mat& yellow_mask, cv::Mat& white_mask) const noexcept;
  /// Extracts the line on the given edges frame using the mask.
  bool extractLine(const cv::Mat& edges, const cv::Mat& mask, Line& line) const noexcept;
  /// Draw the lane between the yellow and white lines.
  void drawLane(const Line& line_yellow, const Line& line_white, cv::Mat& to_draw) const noexcept;

  cv::Mat _roi_mask;
  Line _horizon_line;
  Line _car_line;
  Line _prev_line_yellow;
  Line _prev_line_white;
};