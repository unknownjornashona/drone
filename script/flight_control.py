import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import logging
import os

class FlightController:
    def __init__(self, target_state, kp=1.0, dt=0.1, total_time=5):
        self.kp = kp  # 比例增益
        self.dt = dt  # 采样时间
        self.total_time = total_time  # 总时间
        self.current_state = np.array([0, 0, 0, 0, 0, 0])  # [x, y, z, vx, vy, vz]
        self.target_state = np.array(target_state)  # 目标状态
        self.trajectory = []

        # 设置日志
        logging.basicConfig(filename='flight_control.log', level=logging.INFO,
                            format='%(asctime)s - %(levelname)s - %(message)s')
    
    def control(self):
        try:
            timesteps = int(self.total_time / self.dt)

            for t in range(timesteps):
                state_diff = self.target_state - self.current_state
                
                # 计算控制输入
                control_input = self.kp * state_diff[:3]  # 只考虑位置控制

                # 更新状态
                self.current_state[:3] += self.current_state[3:] * self.dt  # 更新位置
                self.current_state[3:] += control_input * self.dt  # 更新速度
                self.trajectory.append(self.current_state.copy())

                # 限制速度
                self.current_state[3:] = np.clip(self.current_state[3:], -1, 1)

            logging.info("控制完成，最终状态: %s", self.current_state)

        except Exception as e:
            logging.error("发生错误: %s", str(e))
            raise

    def save_trajectory_to_csv(self, filename='trajectory_data.csv'):
        trajectory_array = np.array(self.trajectory)
        df = pd.DataFrame(trajectory_array, columns=['x', 'y', 'z', 'vx', 'vy', 'vz'])
        df.to_csv(filename, index=False)
        logging.info("轨迹数据已保存为 %s", filename)

    def plot_trajectory(self):
        trajectory_array = np.array(self.trajectory)
        plt.figure()
        plt.plot(trajectory_array[:, 0], trajectory_array[:, 1], label='实际轨迹')
        plt.plot(self.target_state[0], self.target_state[1], 'ro', label='目标位置')
        plt.xlabel('X 位置')
        plt.ylabel('Y 位置')
        plt.title('飞行控制轨迹')
        plt.legend()
        plt.grid()
        plt.axis('equal')

        # 保存图表为 PNG 图片
        plt.savefig("trajectory_plot.png")
        plt.close()
        logging.info("图表已保存为 trajectory_plot.png")

# 主程序
if __name__ == "__main__":
    try:
        # 创建飞行控制器实例
        flight_controller = FlightController(target_state=[10, 10, 10])
        
        # 进行控制
        flight_controller.control()
        
        # 保存轨迹到 CSV 文件
        flight_controller.save_trajectory_to_csv()
        
        # 绘制并保存图表
        flight_controller.plot_trajectory()

    except Exception as e:
        print(f"错误: {e}")
