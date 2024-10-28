#include "MultiUAVCoordination.h"

void MultiUAVCoordination::assignTasks(const std::vector<cv::Point>& goals, const std::vector<cv::Point>& uavPositions) {
    const double safetyDistance = 10.0; // 安全距离

    // 记录每个无人机的分配目标
    std::vector<cv::Point> assignedGoals(goals.size(), cv::Point(-1, -1)); // -1 表示未分配
    std::vector<bool> goalAssigned(goals.size(), false); // 目标是否已被分配

    // 遍历无人机
    for (const auto& uavPos : uavPositions) {
        double minDistance = std::numeric_limits<double>::max();
        int bestGoalIndex = -1;

        // 查找与无人机最近的目标
        for (size_t i = 0; i < goals.size(); ++i) {
            if (!goalAssigned[i]) { // 目标未被分配
                double distance = calculateDistance(uavPos, goals[i]);
                if (distance < minDistance) {
                    minDistance = distance;
                    bestGoalIndex = i;
                }
            }
        }

        // 处理分配，并检查碰撞
        if (bestGoalIndex != -1) {
            // 检查是否会与已分配的目标发生碰撞
            bool collision = false;

            for (const auto& assignedGoal : assignedGoals) {
                if (assignedGoal.x != -1 && isCollision(uavPos, assignedGoal, safetyDistance)) {
                    collision = true;
                    break;
                }
            }

            if (!collision) {
                // 分配目标给无人机
                assignedGoals[bestGoalIndex] = goals[bestGoalIndex];
                goalAssigned[bestGoalIndex] = true;
                std::cout << "无人机在 (" << uavPos.x << ", " << uavPos.y << ") 被分配目标 (" 
                          << goals[bestGoalIndex].x << ", " << goals[bestGoalIndex].y << ")" << std::endl;
            } else {
                std::cout << "无人机在 (" << uavPos.x << ", " << uavPos.y << ") 无法分配目标 ("
                          << goals[bestGoalIndex].x << ", " << goals[bestGoalIndex].y << ") - 遇到碰撞。" << std::endl;
            }
        }
    }
}

double MultiUAVCoordination::calculateDistance(const cv::Point& a, const cv::Point& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

bool MultiUAVCoordination::isCollision(const cv::Point& pos1, const cv::Point& pos2, double safetyDistance) {
    return calculateDistance(pos1, pos2) < safetyDistance;
}
