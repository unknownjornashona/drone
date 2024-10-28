import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import logging
import random

class PIDController:
    """ PID 控制器 """
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0

    def update(self, desired, actual, dt):
        error = desired - actual
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class DroneFlightController:
    def __init__(self, target_state, dt=0.1, total_time=20):
        self.dt = dt
        self.total_time = total_time
        self.current_state = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])  # [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.target_state = np.array(target_state)
        self.trajectory = []

        # 初始化 PID 控制器
        self.position_pid = PIDController(1.0, 0.1, 0.01)
        self.attitude_pid = PIDController(1.0, 0.1, 0.01)

        # 设置日志
        logging.basicConfig(filename='drone_flight_control.log', level=logging.INFO,
                            format='%(asctime)s - %(levelname)s - %(message)s')

    def add_wind(self):
        """ 模拟风的影响 """
        self.current_state[3:6] += np.random.normal(0, 0.5, size=3)  # 随机模拟风速

    def dynamic_obstacle_avoidance(self, obstacles):
        """ 简单的动态避障功能 """
        for obs in obstacles:
            distance = np.linalg.norm(self.current_state[:3] - obs[:3])
            if distance < 5.0:  # 如果距离障碍物太近，改变方向
                self.current_state[3:6] = np.array([self.current_state[3] - 1, self.current_state[4] - 1, self.current_state[5]])

    def control(self, obstacles):
        try:
            timesteps = int(self.total_time / self.dt)

            for t in range(timesteps):
                self.add_wind()
                self.dynamic_obstacle_avoidance(obstacles)

                # 对位置进行 PID 控制
                position_control_input = self.position_pid.update(self.target_state[:3], self.current_state[:3], self.dt)

                # 对姿态进行 PID 控制
                attitude_control_input = self.attitude_pid.update(self.target_state[6:], self.current_state[6:], self.dt)

                # 更新状态
                self.current_state[3:6] += position_control_input * self.dt  # 更新速度
                self.current_state[:3] += self.current_state[3:6] * self.dt  # 更新位置
                self.current_state[6:] += attitude_control_input * self.dt  # 更新姿态

                self.trajectory.append(self.current_state.copy())

                # 限制速度与姿态
                self.current_state[3:] = np.clip(self.current_state[3:], -1, 1)
                self.current_state[6:] = np.clip(self.current_state[6:], -np.pi, np.pi)  # 姿态角限制

            logging.info("控制完成，最终状态: %s", self.current_state)

        except Exception as e:
            logging.error("发生错误: %s", str(e))
            raise

    def save_trajectory_to_csv(self, filename='trajectory_data.csv'):
        trajectory_array = np.array(self.trajectory)
        df = pd.DataFrame(trajectory_array, columns=['x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw'])
        df.to_csv(filename, index=False)
        logging.info("轨迹数据已保存为 %s", filename)

    def plot_trajectory(self):
        trajectory_array = np.array(self.trajectory)
        plt.figure(figsize=(12, 6))
        plt.plot(trajectory_array[:, 0], trajectory_array[:, 1], label='实际轨迹')
        plt.plot(self.target_state[0], self.target_state[1], 'ro', label='目标位置')
        plt.xlabel('X 位置')
        plt.ylabel('Y 位置')
        plt.title('无人机飞行控制轨迹')
        plt.legend()
        plt.grid()
        plt.axis('equal')
        plt.savefig("drone_trajectory_plot.png")
        plt.close()
        logging.info("图表已保存为 drone_trajectory_plot.png")

# 主程序
if __name__ == "__main__":
    # 模拟避免障碍物: 随机生成一些障碍物实体
    obstacles = [np.array([random.uniform(0, 10), random.uniform(0, 10), random.uniform(0, 10)]) for _ in range(5)]
    
    try:
        # 创建无人机飞行控制器实例，目标状态包括姿态
        drone_controller = DroneFlightController(target_state=[10, 10, 10, 0, 0, 0])
        
        # 进行控制
        drone_controller.control(obstacles)
        
        # 保存轨迹到 CSV 文件
        drone_controller.save_trajectory_to_csv()
        
        # 绘制并保存图表
        drone_controller.plot_trajectory()

    except Exception as e:
        print(f"错误: {e}")
