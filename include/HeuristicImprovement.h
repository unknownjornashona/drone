#ifndef HEURISTICIMPROVEMENT_H
#define HEURISTICIMPROVEMENT_H

#include <opencv2/opencv.hpp>
#include <string>
#include <cmath>
#include <vector>

class HeuristicImprovement {
public:
    // 评估启发式值的方法
    int evaluateHeuristic(const cv::Point& a, const cv::Point& b, const std::string& method);
    
    // 评估启发式值，考虑障碍物影响
    double evaluateHeuristicWithObstacles(const cv::Point& a, const cv::Point& b, const std::string& method, const std::vector<std::vector<int>>& obstacles);

private:
    // 检查两个点之间是否存在障碍
    bool hasObstacle(const cv::Point& a, const cv::Point& b, const std::vector<std::vector<int>>& obstacles);
};

#endif // HEURISTICIMPROVEMENT_H
