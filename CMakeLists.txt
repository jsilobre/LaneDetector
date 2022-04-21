cmake_minimum_required(VERSION 3.20)

project(ColorLane)

find_package(OpenCV REQUIRED)

add_executable(ColorLane
    "src/main.cpp"
	"src/LaneDetector.cpp"
)

target_link_libraries(ColorLane
    PUBLIC
    ${OpenCV_LIBS}
)

target_include_directories(ColorLane PRIVATE ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})


