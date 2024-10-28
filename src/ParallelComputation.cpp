#include "ParallelComputation.h"
#include <iostream>
#include <omp.h>

void ParallelComputation::optimizePathInParallel(const cv::Mat& mapImg, const cv::Point& start, const cv::Point& end) {
    std::vector<cv::Point> path;
    dijkstra(mapImg, start, end, path);

    // 在这里可以使用路径进行后续处理，例如绘制或输出
    std::cout << "Optimized Path: ";
    for (const auto& point : path) {
        std::cout << "(" << point.x << ", " << point.y << ") ";
    }
    std::cout << std::endl;
}

bool ParallelComputation::isValid(const cv::Point& pt, const cv::Mat& mapImg) {
    return (pt.x >= 0 && pt.x < mapImg.cols && pt.y >= 0 && pt.y < mapImg.rows && mapImg.at<uchar>(pt) == 0);
}

void ParallelComputation::dijkstra(const cv::Mat& mapImg, const cv::Point& start, const cv::Point& end, std::vector<cv::Point>& path) {
    const int directions[4][2] = { {1, 0}, {0, 1}, {-1, 0}, {0, -1} }; // 右、下、左、上
    std::priority_queue<std::pair<int, cv::Point>, std::vector<std::pair<int, cv::Point>>, std::greater<>> pq; // 最小优先队列
    std::vector<std::vector<int>> dist(mapImg.rows, std::vector<int>(mapImg.cols, std::numeric_limits<int>::max()));
    std::vector<std::vector<cv::Point>> prev(mapImg.rows, std::vector<cv::Point>(mapImg.cols, cv::Point(-1, -1)));

    dist[start.y][start.x] = 0;
    pq.emplace(0, start);
    
    while (!pq.empty()) {
        auto [currentDist, currentPoint] = pq.top(); pq.pop();

        if (currentPoint == end) break;

        // 并行计算相邻的点
        #pragma omp parallel for
        for (int i = 0; i < 4; ++i) {
            cv::Point neighbor(currentPoint.x + directions[i][0], currentPoint.y + directions[i][1]);
            if (isValid(neighbor, mapImg)) {
                int newDist = currentDist + 1; // 所有边权重为1
                if (newDist < dist[neighbor.y][neighbor.x]) {
                    dist[neighbor.y][neighbor.x] = newDist;
                    prev[neighbor.y][neighbor.x] = currentPoint;
                    pq.emplace(newDist, neighbor);
                }
            }
        }
    }

    // 从终点重建路径
    for (cv::Point pt = end; pt.x != -1 && pt.y != -1; pt = prev[pt.y][pt.x]) {
        path.push_back(pt);
    }
    std::reverse(path.begin(), path.end()); // 反向路径
}
