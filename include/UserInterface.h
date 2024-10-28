#ifndef USERINTERFACE_H
#define USERINTERFACE_H

#include <opencv2/opencv.hpp>
#include <vector>

class UserInterface {
public:
    // 创建GUI并显示路径
    void createGUI(const cv::Mat& mapImg, const std::vector<cv::Point>& path);
    
    // 路径更新
    void updatePath(const cv::Mat& mapImg, const std::vector<cv::Point>& newPath);

    // 保存当前图像
    void saveImage(const cv::Mat& img, const std::string& filename);
};

#endif // USERINTERFACE_H
