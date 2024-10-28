#ifndef WEATHERMODELING_H
#define WEATHERMODELING_H

#include <opencv2/opencv.hpp>
#include <vector>

class WeatherModeling {
public:
    // 存储天气信息的结构体
    struct WeatherCondition {
        float windSpeed; // 风速，单位 m/s
        float windDirection; // 风向，单位度
        float temperature; // 温度，单位摄氏度
    };

    // 设置天气条件
    void setWeatherConditions(float windSpeed, float windDirection, float temperature);

    // 模拟天气对路径的影响
    std::vector<cv::Point> modelWeatherEffects(const std::vector<cv::Point>& path);

private:
    WeatherCondition currentConditions; // 当前天气条件
};

#endif // WEATHERMODELING_H
