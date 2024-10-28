#include "UserInterface.h"
#include <iostream>

void UserInterface::createGUI(const cv::Mat& mapImg, const std::vector<cv::Point>& path) {
    // 创建一个深拷贝的地图图像以进行绘制
    cv::Mat displayImg = mapImg.clone();

    // 在地图上绘制路径
    for (size_t i = 0; i < path.size() - 1; ++i) {
        cv::line(displayImg, path[i], path[i + 1], cv::Scalar(0, 255, 0), 2); // 绘制绿色路径
    }

    // 在路径点上绘制圆点
    for (const auto& pt : path) {
        cv::circle(displayImg, pt, 3, cv::Scalar(255, 0, 0), -1); // 绘制蓝色路径点
    }

    // 显示地图图像
    cv::imshow("Drone Path Visualization", displayImg);

    // 等待用户按键以关闭窗口
    std::cout << "Press 's' to save the image, or any other key to close the window..." << std::endl;

    // 等待用户输入
    char key = cv::waitKey(0);
    if (key == 's') {
        saveImage(displayImg, "path_visualization.png");
        std::cout << "Image saved as 'path_visualization.png'." << std::endl;
    }

    cv::destroyAllWindows(); // 销毁所有窗口
}

void UserInterface::updatePath(const cv::Mat& mapImg, const std::vector<cv::Point>& newPath) {
    // 可以选择实现一个动态更新的GUI
    createGUI(mapImg, newPath); // 重新调用创建GUI
}

void UserInterface::saveImage(const cv::Mat& img, const std::string& filename) {
    cv::imwrite(filename, img); // 保存图像
}
