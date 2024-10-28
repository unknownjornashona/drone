#ifndef PATHSMOOTHING_H
#define PATHSMOOTHING_H

#include <vector>
#include <opencv2/opencv.hpp>

class PathSmoothing {
public:
    std::vector<cv::Point> smoothPath(const std::vector<cv::Point>& path);
};

#endif // PATHSMOOTHING_H
