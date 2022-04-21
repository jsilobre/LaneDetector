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
  void computeMask(const cv::Size size);
  void colorMask(cv::Mat& frame, cv::Mat& yellow_mask, cv::Mat& white_mask) const noexcept;
  bool extractLine(const cv::Mat& edges, const cv::Mat& mask, Line& line) const noexcept;
  void resizeLines(Line& line_yellow, Line& line_white, const size_t max_heigh) const noexcept;

  cv::Mat _roi_mask;
  Line _prev_line_yellow;
  Line _prev_line_white;
};