#ifndef MACHINELEARNINGINTEGRATION_H
#define MACHINELEARNINGINTEGRATION_H

#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp> // OpenCV 机器学习库
#include <vector>
#include <iostream>

class MachineLearningIntegration {
public:
    // 训练机器学习模型
    void trainModel(const std::vector<cv::Point>& trainingData, const std::vector<int>& labels);

    // 使用模型进行预测
    int predict(const cv::Point& testPoint);

private:
    cv::Ptr<cv::ml::KNearest> knn; // KNN 模型指针
};

#endif // MACHINELEARNINGINTEGRATION_H
