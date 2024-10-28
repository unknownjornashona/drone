#ifndef DYNAMICPATHPLANNER_H
#define DYNAMICPATHPLANNER_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>
#include <limits>
#include <iostream>

class DynamicPathPlanner {
public:
    // 更新障碍物
    void updateObstacles(const cv::Mat& mapImg);
    
    // 路径重新规划
    void replanning(cv::Point currentPos, cv::Point goal);

private:
    // 当前的障碍物地图
    cv::Mat currentMap;
    
    // 计算路径（使用 Dijkstra 或 A* 算法）
    void findPath(const cv::Point& start, const cv::Point& end, std::vector<cv::Point>& path);
    
    // 辅助方法，计算距离
    double calculateDistance(const cv::Point& a, const cv::Point& b);
    
    // 检查是否为有效坐标
    bool isValid(const cv::Point& pt, const cv::Mat& mapImg);
};

#endif // DYNAMICPATHPLANNER_H
