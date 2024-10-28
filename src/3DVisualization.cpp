#include "ThreeDVisualization.h"
#include <iostream>
#include <GL/glut.h> // 确保已安装 GLUT

ThreeDVisualization::ThreeDVisualization() : window(nullptr) {}

ThreeDVisualization::~ThreeDVisualization() {
    glfwTerminate();
}

void ThreeDVisualization::initGL() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return;
    }

    // 创建窗口
    window = glfwCreateWindow(800, 600, "3D UAV Visualization", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window);
    
    glewInit();
    glEnable(GL_DEPTH_TEST); // 开启深度测试
}

void ThreeDVisualization::setupCamera() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 800.0 / 600.0, 0.1, 100.0);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0, -20.0, -50.0); // 设置初始视角
}

void ThreeDVisualization::drawPath(const std::vector<cv::Point3f>& path) {
    glColor3f(0.0f, 1.0f, 0.0f); // 绿色路径
    glBegin(GL_LINE_STRIP);
    for (const auto& point : path) {
        glVertex3f(point.x, point.y, point.z);
    }
    glEnd();
}

void ThreeDVisualization::drawDrone(const cv::Point3f& position) {
    glPushMatrix();
    glTranslatef(position.x, position.y, position.z);
    glColor3f(1.0f, 0.0f, 0.0f); // 无人机为红色
    glutWireSphere(1.0, 10, 10); // 绘制一个红色的线框球体作为无人机
    glPopMatrix();
}

void ThreeDVisualization::display(const std::vector<cv::Point3f>& path, const cv::Point3f& dronePosition) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    setupCamera();

    drawPath(path);
    drawDrone(dronePosition);

    glfwSwapBuffers(window);
    glfwPollEvents();
}

void ThreeDVisualization::run() {
    std::vector<cv::Point3f> path = {
        cv::Point3f(0, 0, 0),
        cv::Point3f(1, 2, 1),
        cv::Point3f(2, 4, 2),
        cv::Point3f(3, 3, 3),
        cv::Point3f(4, 5, 4)
    };
    cv::Point3f dronePosition(3, 3, 3); // 设置初始无人机位置

    while (!glfwWindowShouldClose(window)) {
        display(path, dronePosition);
    }
}
