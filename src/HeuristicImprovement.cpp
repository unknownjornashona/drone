#include "HeuristicImprovement.h"

int HeuristicImprovement::evaluateHeuristic(const cv::Point& a, const cv::Point& b, const std::string& method) {
    if (method == "manhattan") {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    } else if (method == "euclidean") {
        return static_cast<int>(std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2)));
    } else if (method == "chebyshev") {
        return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
    } else if (method == "diagonal") {
        // 计算对角线距离
        return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
    } else {
        return 0; // 不支持的方法
    }
}

double HeuristicImprovement::evaluateHeuristicWithObstacles(const cv::Point& a, const cv::Point& b, const std::string& method, const std::vector<std::vector<int>>& obstacles) {
    // 使用启发式评估两个点之间的距离
    double heuristicValue = evaluateHeuristic(a, b, method);
    
    // 检查路径中是否有障碍，并调整启发式值
    if (hasObstacle(a, b, obstacles)) {
        heuristicValue += 1000; // 添加高额惩罚，表示不可达
    }

    return heuristicValue;
}

bool HeuristicImprovement::hasObstacle(const cv::Point& a, const cv::Point& b, const std::vector<std::vector<int>>& obstacles) {
    // 简单线性检查障碍，假定障碍物用0表示可通行，1表示障碍
    int dx = std::abs(b.x - a.x);
    int dy = std::abs(b.y - a.y);
    int sx = (a.x < b.x) ? 1 : -1;
    int sy = (a.y < b.y) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (a.x < 0 || a.x >= obstacles.size() || a.y < 0 || a.y >= obstacles[0].size()) {
            return false; // 超过边界
        }
        if (obstacles[a.x][a.y] == 1) {
            return true; // 遇到障碍
        }
        if (a.x == b.x && a.y == b.y) {
            break; // 达到目标点
        }
        int e2 = err * 2;
        if (e2 > -dy) { err -= dy; a.x += sx; }
        if (e2 < dx)   { err += dx; a.y += sy; }
    }
    return false; // 路径可通行
}
