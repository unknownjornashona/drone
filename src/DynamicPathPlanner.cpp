#include "DynamicPathPlanner.h"

void DynamicPathPlanner::updateObstacles(const cv::Mat& mapImg) {
    currentMap = mapImg.clone(); // 更新当前的障碍物地图
}

void DynamicPathPlanner::replanning(cv::Point currentPos, cv::Point goal) {
    std::vector<cv::Point> path;
    findPath(currentPos, goal, path);

    // 打印新的路径
    std::cout << "重新规划的路径: ";
    for (const auto& point : path) {
        std::cout << "(" << point.x << ", " << point.y << ") ";
    }
    std::cout << std::endl;
}

void DynamicPathPlanner::findPath(const cv::Point& start, const cv::Point& end, std::vector<cv::Point>& path) {
    const int directions[4][2] = { {1, 0}, {0, 1}, {-1, 0}, {0, -1} }; // 右、下、左、上
    std::priority_queue<std::pair<double, cv::Point>, std::vector<std::pair<double, cv::Point>>, std::greater<>> pq; // 最小优先队列
    std::vector<std::vector<double>> dist(currentMap.rows, std::vector<double>(currentMap.cols, std::numeric_limits<double>::max()));
    std::vector<std::vector<cv::Point>> prev(currentMap.rows, std::vector<cv::Point>(currentMap.cols, cv::Point(-1, -1)));

    dist[start.y][start.x] = 0;
    pq.emplace(0, start);
    
    while (!pq.empty()) {
        auto [currentDist, currentPoint] = pq.top();
        pq.pop();

        if (currentPoint == end) break;

        for (const auto& dir : directions) {
            cv::Point neighbor(currentPoint.x + dir[0], currentPoint.y + dir[1]);
            if (isValid(neighbor, currentMap)) {
                double newDist = currentDist + calculateDistance(currentPoint, neighbor);
                if (newDist < dist[neighbor.y][neighbor.x]) {
                    dist[neighbor.y][neighbor.x] = newDist;
                    prev[neighbor.y][neighbor.x] = currentPoint;
                    pq.emplace(newDist, neighbor);
                }
            }
        }
    }

    // 重建路径
    for (cv::Point pt = end; pt.x != -1 && pt.y != -1; pt = prev[pt.y][pt.x]) {
        path.push_back(pt);
    }
    std::reverse(path.begin(), path.end()); // 反向路径
}

double DynamicPathPlanner::calculateDistance(const cv::Point& a, const cv::Point& b) {
    return cv::norm(a - b); // 使用欧几里得距离
}

bool DynamicPathPlanner::isValid(const cv::Point& pt, const cv::Mat& mapImg) {
    return (pt.x >= 0 && pt.x < mapImg.cols && pt.y >= 0 && pt.y < mapImg.rows && mapImg.at<uchar>(pt) == 0);
}
