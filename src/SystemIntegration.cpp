#include "SystemIntegration.h"

void SystemIntegration::integrateWithFlightControl(const cv::Point& start, const cv::Point& goal, const cv::Mat& mapImg) {
    std::vector<cv::Point> optimizedPath;

    // 使用路径优化器计算优化路径
    pathOptimizer.findOptimalPath(start, goal, mapImg, optimizedPath);

    // 将优化后的路径传递给飞行控制系统
    for (const auto& point : optimizedPath) {
        // 模拟飞行控制系统执行的过程
        std::cout << "飞行控制系统指示飞往目标点: (" << point.x << ", " << point.y << ")" << std::endl;
        // 在实际应用中，这里应调用飞行控制API来移动无人机
    }
}
