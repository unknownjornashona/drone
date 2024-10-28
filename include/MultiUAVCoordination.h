#ifndef MULTIUAVCOORDINATION_H
#define MULTIUAVCOORDINATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <utility>
#include <algorithm>
#include <iostream>

class MultiUAVCoordination {
public:
    // 分配任务给多架无人机
    void assignTasks(const std::vector<cv::Point>& goals, const std::vector<cv::Point>& uavPositions);

private:
    // 计算无人机与目标之间的距离
    double calculateDistance(const cv::Point& a, const cv::Point& b);
    
    // 检查两架无人机之间是否会发生碰撞
    bool isCollision(const cv::Point& pos1, const cv::Point& pos2, double safetyDistance);
};

#endif // MULTIUAVCOORDINATION_H
