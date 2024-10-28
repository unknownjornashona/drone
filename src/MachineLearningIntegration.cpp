#include "MachineLearningIntegration.h"

void MachineLearningIntegration::trainModel(const std::vector<cv::Point>& trainingData, const std::vector<int>& labels) {
    // 将训练数据转换为矩阵格式
    cv::Mat trainData(trainingData.size(), 2, CV_32F);
    for (size_t i = 0; i < trainingData.size(); ++i) {
        trainData.at<float>(i, 0) = trainingData[i].x;
        trainData.at<float>(i, 1) = trainingData[i].y;
    }

    // 将标签转换为矩阵格式
    cv::Mat labelsMat(labels.size(), 1, CV_32S);
    for (size_t i = 0; i < labels.size(); ++i) {
        labelsMat.at<int>(i, 0) = labels[i];
    }

    // 初始化并训练 KNN 模型
    knn = cv::ml::KNearest::create();
    knn->setDefaultK(3); // 设置近邻数
    knn->train(trainData, cv::ml::ROW_SAMPLE, labelsMat);
    std::cout << "模型训练完成。" << std::endl;
}

int MachineLearningIntegration::predict(const cv::Point& testPoint) {
    // 将测试点转换为矩阵格式
    cv::Mat testData(1, 2, CV_32F);
    testData.at<float>(0, 0) = testPoint.x;
    testData.at<float>(0, 1) = testPoint.y;

    // 执行预测
    cv::Mat result;
    knn->findNearest(testData, knn->getDefaultK(), result);
    return static_cast<int>(result.at<float>(0, 0)); // 返回预测标签
}
