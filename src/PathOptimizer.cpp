#include "PathOptimizer.h"

void PathOptimizer::findOptimalPath(const cv::Point& start, const cv::Point& goal, const cv::Mat& mapImg, std::vector<cv::Point>& path) {
    // 使用简单的Dijkstra算法实现路径优化
    const int directions[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}}; // 右、下、左、上
    std::priority_queue<std::pair<double, cv::Point>, std::vector<std::pair<double, cv::Point>>, std::greater<>> pq; // 最小优先队列
    std::vector<std::vector<double>> dist(mapImg.rows, std::vector<double>(mapImg.cols, std::numeric_limits<double>::max()));
    std::vector<std::vector<cv::Point>> prev(mapImg.rows, std::vector<cv::Point>(mapImg.cols, cv::Point(-1, -1)));

    dist[start.y][start.x] = 0;
    pq.emplace(0, start);

    while (!pq.empty()) {
        auto [currentDist, currentPoint] = pq.top(); 
        pq.pop();

        if (currentPoint == goal) break;

        for (const auto& dir : directions) {
            cv::Point neighbor(currentPoint.x + dir[0], currentPoint.y + dir[1]);
            if (neighbor.x >= 0 && neighbor.x < mapImg.cols && neighbor.y >= 0 && neighbor.y < mapImg.rows && mapImg.at<uchar>(neighbor) == 0) {
                double newDist = currentDist + 1; // 假设每步的代价为1
                if (newDist < dist[neighbor.y][neighbor.x]) {
                    dist[neighbor.y][neighbor.x] = newDist;
                    prev[neighbor.y][neighbor.x] = currentPoint;
                    pq.emplace(newDist, neighbor);
                }
            }
        }
    }

    // 从终点反向重建路径
    for (cv::Point pt = goal; pt.x != -1 && pt.y != -1; pt = prev[pt.y][pt.x]) {
        path.push_back(pt);
    }
    std::reverse(path.begin(), path.end()); // 反转路径
}
