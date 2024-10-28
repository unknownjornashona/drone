#ifndef DRONEPATHOPTIMIZER_H
#define DRONEPATHOPTIMIZER_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <queue>
#include <utility>

class DronePathOptimizer {
public:
    DronePathOptimizer(const std::string& logFile);
    void optimizePath(const cv::Mat& mapImg, cv::Point start, cv::Point goal);
    
private:
    struct Node {
        cv::Point pos;
        int g, h, f;
        Node* parent;
        
        Node(cv::Point p, int gCost, int hCost)
            : pos(p), g(gCost), h(hCost), f(gCost + hCost), parent(nullptr) {}
        
        bool operator>(const Node& other) const {
            return f > other.f;
        }
    };
    
    void log(const std::string& message);
    void throwError(const std::string& message);
    int heuristic(const cv::Point& a, const cv::Point& b);
    std::vector<cv::Point> reconstructPath(Node* node);
    
    std::ofstream logStream;
};

#endif // DRONEPATHOPTIMIZER_H
