#include "SensorIntegration.h"

void SensorIntegration::setSensorWeight(const std::string& sensorType, double weight) {
    sensorWeights[sensorType] = weight;
}

void SensorIntegration::setKalmanNoiseParams(const cv::Mat& processNoiseCov, const cv::Mat& measurementNoiseCov) {
    this->processNoiseCov = processNoiseCov.clone();
    this->measurementNoiseCov = measurementNoiseCov.clone();
}

void SensorIntegration::processSensorData(const cv::Mat& sensorData, const std::string& sensorType) {
    if (sensorData.empty()) {
        return; // 确保传感器数据非空
    }

    // 仅处理灰度图像
    cv::Mat grayData;
    if (sensorData.channels() == 3) {
        cv::cvtColor(sensorData, grayData, cv::COLOR_BGR2GRAY);
    } else {
        grayData = sensorData.clone();
    }

    // 获取传感器权重，如果未设置则使用默认权重
    double weight = sensorWeights.count(sensorType) ? sensorWeights[sensorType] : defaultWeight;

    // 加权平均融合处理
    weightedFusion(grayData, weight);

    // 卡尔曼滤波处理
    cv::Mat measurement = (cv::Mat_<double>(2, 1) << grayData.at<uchar>(grayData.rows / 2, grayData.cols / 2), // 测量中心值
                                                     grayData.at<uchar>(grayData.rows / 2, grayData.cols / 2)); // 另一测量
    cv::Mat filteredOutput = kalmanFilter(measurement);

    // 记录输入传感器数据和融合输出
    logData(grayData, filteredOutput, "sensor_fusion_log.txt");
}

void SensorIntegration::weightedFusion(const cv::Mat& sensorData, double weight) {
    if (fusedMap.empty()) {
        fusedMap = sensorData.clone(); // 如果没有融合图像，则初始化
    } else {
        // 进行加权平均融合
        cv::Mat weightedMap = (fusedMap * (1 - weight) + sensorData * weight);
        fusedMap = weightedMap.clone();
    }
}

cv::Mat SensorIntegration::kalmanFilter(const cv::Mat& measurement) {
    if (!kfInitialized) {
        kf.init(4, 2); // 4状态，2观测
        setIdentity(kf.measurementMatrix); // 观测矩阵

        // 初始化卡尔曼滤波器
        setIdentity(processNoiseCov, cv::Scalar(1e-4)); // 过程噪声 covariance
        setIdentity(measurementNoiseCov, cv::Scalar(0.1)); // 测量噪声 covariance

        kf.processNoiseCov = processNoiseCov;
        kf.measurementNoiseCov = measurementNoiseCov;
        
        kfInitialized = true; // 标记为已初始化
    }

    // 更新卡尔曼滤波器
    kf.predict();
    return kf.correct(measurement); // 返回更新后的状态
}

void SensorIntegration::logData(const cv::Mat& sensorData, const cv::Mat& fusedData, const std::string& filename) {
    std::ofstream logFile(filename, std::ios::app);
    if (!logFile.is_open()) {
        std::cerr << "Error: Cannot open log file." << std::endl;
        return;
    }

    // 记录传感器数据
    logFile << "Sensor Data:\n" << sensorData << "\n";
    logFile << "Fused Output:\n" << fusedData << "\n";
    logFile << "-------------------------------------\n";
    logFile.close();
}

cv::Mat SensorIntegration::getFusedMap() const {
    return fusedMap; // 返回融合后的地图
}
