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
  void computeMask(const cv::Size size) noexcept;
  void computeBasicLine(const cv::Mat & frame) noexcept;
  bool computeIntersection(const Line& line1, const Line& line2, cv::Point &intersection_point) const noexcept;
  void colorMask(cv::Mat& frame, cv::Mat& yellow_mask, cv::Mat& white_mask) const noexcept;
  bool extractLine(const cv::Mat& edges, const cv::Mat& mask, Line& line) const noexcept;
  void drawLane(const Line& line_yellow, const Line& line_white, cv::Mat& to_draw) const noexcept;

  cv::Mat _roi_mask;
  Line _horizon_line;
  Line _car_line;
  Line _prev_line_yellow;
  Line _prev_line_white;
};