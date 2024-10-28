#ifndef SENSORINTEGRATION_H
#define SENSORINTEGRATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <string>
#include <fstream>

class SensorIntegration {
public:
    // 设置传感器权重
    void setSensorWeight(const std::string& sensorType, double weight);

    // 设置卡尔曼滤波器的噪声参数
    void setKalmanNoiseParams(const cv::Mat& processNoiseCov, const cv::Mat& measurementNoiseCov);

    // 处理并融合传感器数据，输入为传感器数据图像和类型
    void processSensorData(const cv::Mat& sensorData, const std::string& sensorType);

    // 获取融合后的环境地图
    cv::Mat getFusedMap() const;

private:
    cv::Mat fusedMap; // 用于存储融合后的地图
    std::map<std::string, double> sensorWeights; // 储存传感器的权重
    double defaultWeight = 0.5; // 默认权重
    cv::KalmanFilter kf; // 卡尔曼滤波器
    bool kfInitialized = false; // 卡尔曼滤波器是否初始化

    // 噪声参数
    cv::Mat processNoiseCov; // 过程噪声协方差
    cv::Mat measurementNoiseCov; // 测量噪声协方差

    // 加权平均融合
    void weightedFusion(const cv::Mat& sensorData, double weight);

    // 卡尔曼滤波器处理
    cv::Mat kalmanFilter(const cv::Mat& measurement);

    // 保存数据到文件
    void logData(const cv::Mat& sensorData, const cv::Mat& fusedData, const std::string& filename);
};

#endif // SENSORINTEGRATION_H
