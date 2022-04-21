#include "LaneDetector.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

// create a mask to focus on the road part of the image
void LaneDetector::computeMask(const cv::Size size) noexcept {
  _roi_mask = cv::Mat::zeros(size, CV_8UC1);

  for (size_t i = 0; i < _roi_mask.rows; ++i) {
    for (size_t j = 0; j < _roi_mask.cols; ++j) {
      if (i > _roi_mask.rows * 0.60 && (j > _roi_mask.cols / 5) && (j < 4 * _roi_mask.cols / 5)) {
        _roi_mask.at<uchar>(i, j) = 255;
      }

    }
  }
  cv::cvtColor(_roi_mask, _roi_mask, cv::COLOR_GRAY2BGR);
}

void LaneDetector::computeBasicLine(const cv::Mat & frame) noexcept {
  _horizon_line = Line(cv::Point(0, frame.rows * 0.65), cv::Point(frame.cols - 1, frame.rows * 0.65));
  _car_line = Line(cv::Point(0, frame.rows * 0.95), cv::Point(frame.cols - 1, frame.rows * 0.95));
}

bool LaneDetector::computeIntersection(const Line& line1, const Line& line2, cv::Point &intersection_point) const noexcept
{
  cv::Point x = line2.p1 - line1.p1;
  cv::Point d1 = line1.p2 - line1.p1;
  cv::Point d2 = line2.p2 - line2.p1;

  float cross = d1.x*d2.y - d1.y*d2.x;
  if (std::abs(cross) < 1e-8)
    return false;

  double t1 = (x.x * d2.y - x.y * d2.x) / cross;
  intersection_point = line1.p1 + d1 * t1;
  
  return true;
}

void LaneDetector::processFrame(const cv::Mat& input_frame) {
  cv::Mat resized;
  cv::resize(input_frame, resized, input_frame.size() / 2);
  
  // compute mask and horizon line on the first frame
  if (_roi_mask.empty()) computeMask(resized.size());
  if (_horizon_line.norm == 0) computeBasicLine(resized);

  cv::Mat masked, edges;

  // apply the ROI mask
  cv::bitwise_and(resized, _roi_mask, masked);

  // extract the lane lines by colors
  cv::Mat yellow_mask, white_mask;
  colorMask(masked, yellow_mask, white_mask);
  
  // apply some blur before edge detection
  cv::GaussianBlur(masked, masked, cv::Size(5, 5), 0, 0);
  cv::Canny(masked, edges, 50, 150);
   
  // extract the two lines, or use the previous one if not detected
  Line line_yellow, line_white;
  if (extractLine(edges, yellow_mask, line_yellow)) {
    _prev_line_yellow = line_yellow;    
  }
  else {
    line_yellow = _prev_line_yellow;
  }

  if (extractLine(edges, white_mask, line_white)) {
    _prev_line_white = line_white;
  }
  else {
    line_white = _prev_line_white;
  }

  // plot results
  cv::Mat lane_print = cv::Mat::zeros(resized.size(), resized.type());
  drawLane(line_yellow, line_white, lane_print);

  cv::Mat final_result;
  cv::addWeighted(resized, 0.8, lane_print, 1.0, 0, final_result);
  
  cv::resize(final_result, final_result, input_frame.size());
  cv::imshow("Lane", final_result);
 
}

void LaneDetector::colorMask(cv::Mat& frame, cv::Mat& yellow_mask, cv::Mat& white_mask) const noexcept {
  cv::Mat grayscale, HSV, HLS ;

  // Get the grayscale and HSV
  cv::cvtColor(frame, HSV, cv::COLOR_BGR2HSV);
  cv::cvtColor(frame, grayscale, cv::COLOR_BGR2GRAY);

  // yellow thresold
  cv::inRange(HSV, cv::Scalar(20, 90, 90), cv::Scalar(50, 255, 255), yellow_mask);

  // white thresold 
  cv::inRange(grayscale, 190, 255, white_mask);
}

bool LaneDetector::extractLine(const cv::Mat& edges, const cv::Mat& mask, Line& line) const noexcept{
  cv::Mat masked_edges;
  cv::bitwise_and(edges, mask, masked_edges);

  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(masked_edges, lines, 5 /*rho*/, CV_PI / 180 /*theta*/, 50 /*threshold*/, 30 /*minLineLength*/, 10 /*maxLineGap*/);
  
  // get the longest line
  line = Line(); 
  for (const auto l : lines) {
    auto p1_tmp = cv::Point(l[0], l[1]);
    auto p2_tmp = cv::Point(l[2], l[3]);
    auto norm = cv::norm(p1_tmp - p2_tmp);
    if (norm > line.norm) {
      line.p1 = p1_tmp;
      line.p2 = p2_tmp;
      line.norm = norm;
    }
  }

  if (line.norm > 0) {
    return true;
  }

  return false;
}

void LaneDetector::drawLane(const Line& line_yellow, const Line& line_white, cv::Mat& to_draw) const noexcept {
  if (line_white.norm != 0 && line_yellow.norm != 0) {
    std::vector<cv::Point> points;

    cv::Point origin_point_yellow, horizon_point_yellow;
    cv::Point origin_point_white, horizon_point_white;

    if (computeIntersection(_horizon_line, line_yellow, horizon_point_yellow)) {
      points.push_back(horizon_point_yellow);
    }
    else {
      points.push_back(line_yellow.p2);
    }
    if (computeIntersection(_car_line, line_yellow, origin_point_yellow)) {
      points.push_back(origin_point_yellow);
    }
    else {
      points.push_back(line_yellow.p1);
    }

    if (computeIntersection(_car_line, line_white, origin_point_white)) {
      points.push_back(origin_point_white);
    }
    else {
      points.push_back(line_white.p2);
    }
    if (computeIntersection(_horizon_line, line_white, horizon_point_white)) {
      points.push_back(horizon_point_white);
    }
    else {
      points.push_back(line_white.p1);
    }
   
    // draw the lane as a polyline
    cv::fillConvexPoly(to_draw, points, cv::Scalar(255, 0, 0), cv::LINE_AA);
  }
}