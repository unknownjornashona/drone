#include "WeatherModeling.h"
#include <cmath> // for std::cos, std::sin

void WeatherModeling::setWeatherConditions(float windSpeed, float windDirection, float temperature) {
    currentConditions.windSpeed = windSpeed;
    currentConditions.windDirection = windDirection;
    currentConditions.temperature = temperature;
}

std::vector<cv::Point> WeatherModeling::modelWeatherEffects(const std::vector<cv::Point>& path) {
    std::vector<cv::Point> affectedPath;

    for (const auto& point : path) {
        // 计算受风速影响的路径偏移
        float windEffectX = currentConditions.windSpeed * std::cos(currentConditions.windDirection * M_PI / 180.0);
        float windEffectY = currentConditions.windSpeed * std::sin(currentConditions.windDirection * M_PI / 180.0);
        
        // 新位置考虑风速的影响
        cv::Point newPoint = point + cv::Point(static_cast<int>(windEffectX), static_cast<int>(windEffectY));
        affectedPath.push_back(newPoint);
    }

    return affectedPath;
}
