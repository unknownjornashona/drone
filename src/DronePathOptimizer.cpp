#include "DronePathOptimizer.h"

DronePathOptimizer::DronePathOptimizer(const std::string& logFile) {
    logStream.open(logFile, std::ios::app);
    if (!logStream.is_open()) {
        throwError("Failed to open log file.");
    }
}

void DronePathOptimizer::optimizePath(const cv::Mat& mapImg, cv::Point start, cv::Point goal) {
    if (mapImg.empty()) {
        throwError("Input map image is empty.");
    }

    log("Starting path optimization...");

    std::priority_queue<Node*, std::vector<Node*>, std::greater<Node*>> openSet;
    std::vector<Node*> allNodes;
    
    Node* startNode = new Node(start, 0, heuristic(start, goal));
    openSet.push(startNode);
    allNodes.push_back(startNode);
    
    std::vector<cv::Point> directions = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} };

    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();

        if (current->pos == goal) {
            auto path = reconstructPath(current);
            // Draw the path on the image
            for (const auto& p : path) {
                mapImg.at<uchar>(p) = 128; // Mark path in gray
            }
            log("Path optimization completed.");
            cv::imwrite("path_image.png", mapImg); // Save path image
            return;
        }

        for (const auto& dir : directions) {
            cv::Point neighbor = current->pos + dir;
            if (neighbor.x < 0 || neighbor.x >= mapImg.rows || neighbor.y < 0 || neighbor.y >= mapImg.cols) continue;
            if (mapImg.at<uchar>(neighbor) == 255) continue; // Impassable

            int gCost = current->g + 1;
            int hCost = heuristic(neighbor, goal);
            Node* neighborNode = new Node(neighbor, gCost, hCost);
            neighborNode->parent = current;

            // Check if this path to neighbor is better
            if (std::find_if(allNodes.begin(), allNodes.end(), [&](Node* n) { return n->pos == neighbor; }) != allNodes.end()) {
                continue; // Already visited
            }

            openSet.push(neighborNode);
            allNodes.push_back(neighborNode);
        }
    }

    throwError("No path found to the goal.");
}

int DronePathOptimizer::heuristic(const cv::Point& a, const cv::Point& b) {
    return abs(a.x - b.x) + abs(a.y - b.y); // Manhattan distance
}

std::vector<cv::Point> DronePathOptimizer::reconstructPath(Node* node) {
    std::vector<cv::Point> path;
    while (node) {
        path.push_back(node->pos);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void DronePathOptimizer::log(const std::string& message) {
    logStream << message << std::endl;
}

void DronePathOptimizer::throwError(const std::string& message) {
    logStream << "Error: " << message << std::endl;
    logStream.close();
    throw std::runtime_error(message);
}
