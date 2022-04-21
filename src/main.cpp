
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#include "LaneDetector.h"
#include "Chrono.hpp"


int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cout << "Input video path needed." << std::endl;
    return -1;
  }

  cv:: VideoCapture input_video(argv[1]);  
  if (!input_video.isOpened())
  {
    std::cout << "Could not open the input video." << std::endl;
    return -1;
  }
  
  LaneDetector lane_detector;
  Chrono chrono;

  cv::Mat input_frame;
  int nb_frame = 0;
  for (;;) {
    input_video >> input_frame;
    if (input_frame.empty()) break;

    lane_detector.processFrame(input_frame);

    cv::waitKey(1);

    nb_frame++;
  }
  std::cout << "Finished processing "<< nb_frame << " frames in " << chrono.total() << "ms" << std::endl;
  return 0;
}