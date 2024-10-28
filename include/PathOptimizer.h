#ifndef PATHOPTIMIZER_H
#define PATHOPTIMIZER_H

#include <opencv2/opencv.hpp>
#include <vector>

// 假设的路径优化类
class PathOptimizer {
public:
    // 找到最优路径的公共接口
    void findOptimalPath(const cv::Point& start, const cv::Point& goal, const cv::Mat& mapImg, std::vector<cv::Point>& path);
};

#endif // PATHOPTIMIZER_H
