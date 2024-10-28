#ifndef PARALLELCOMPUTATION_H
#define PARALLELCOMPUTATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>
#include <limits>

class ParallelComputation {
public:
    // 在并行中优化路径
    void optimizePathInParallel(const cv::Mat& mapImg, const cv::Point& start, const cv::Point& end);

private:
    // 辅助方法，检查是否合法的坐标
    bool isValid(const cv::Point& pt, const cv::Mat& mapImg);
    
    // 计算路径（例如使用 Dijkstra 算法）
    void dijkstra(const cv::Mat& mapImg, const cv::Point& start, const cv::Point& end, std::vector<cv::Point>& path);
};

#endif // PARALLELCOMPUTATION_H
