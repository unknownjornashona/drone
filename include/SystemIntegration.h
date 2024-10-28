#ifndef SYSTEMINTEGRATION_H
#define SYSTEMINTEGRATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
// 假设的路径优化类
#include "PathOptimizer.h" 

class SystemIntegration {
public:
    // 将路径优化算法与飞行控制系统集成
    void integrateWithFlightControl(const cv::Point& start, const cv::Point& goal, const cv::Mat& mapImg);

private:
    PathOptimizer pathOptimizer; // 路径优化器实例
};

#endif // SYSTEMINTEGRATION_H
