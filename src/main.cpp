
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#include "LaneDetector.h"
#include "Chrono.hpp"


int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cout << "Input video path needed." << std::endl;
    return -1;
  }

  cv::VideoCapture input_video(argv[1]);  

  bool generate_video{ false };
  if(argc > 2 && std::string(argv[2]) == std::string("VIDEO")) {
    generate_video = true;
  }

  if (!input_video.isOpened())
  {
    std::cout << "Could not open the input video." << std::endl;
    return -1;
  }
  
  LaneDetector lane_detector;
  Chrono chrono;

  cv::VideoWriter output_video;
  if (generate_video) {
    auto size = cv::Size((int)input_video.get(cv::CAP_PROP_FRAME_WIDTH), (int)input_video.get(cv::CAP_PROP_FRAME_HEIGHT));
    output_video.open("ouput.mp4", static_cast<int>(input_video.get(cv::CAP_PROP_FOURCC)), input_video.get(cv::CAP_PROP_FPS), size, true);
  }
  cv::Mat input_frame, output_frame;
  int nb_frame = 0;
  for (;;) {
    input_video >> input_frame;
    if (input_frame.empty()) break;

    lane_detector.processFrame(input_frame, output_frame);
    if (generate_video) {
      output_video << output_frame;
    }
    else {
      cv::imshow("Lane", output_frame);
      cv::waitKey(1);
    } 

    nb_frame++;
  }
  if (generate_video) {
    output_video.release();
  }
  std::cout << "Finished processing "<< nb_frame << " frames in " << chrono.total() << "ms" << std::endl;
  return 0;
}