#ifndef THREEDVISUALIZATION_H
#define THREEDVISUALIZATION_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <opencv2/opencv.hpp>

class ThreeDVisualization {
public:
    ThreeDVisualization();
    ~ThreeDVisualization();
    
    void initGL();

    void display(const std::vector<cv::Point3f>& path, const cv::Point3f& dronePosition);

    void run();

private:
    GLFWwindow* window;

    void setupCamera();
    void drawPath(const std::vector<cv::Point3f>& path);
    void drawDrone(const cv::Point3f& position);
};

#endif // THREEDVISUALIZATION_H
