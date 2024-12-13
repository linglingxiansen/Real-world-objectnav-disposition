import math
import numpy as np

class IMUConverter:
    def __init__(self):
        self.prev_quaternion = None
        self.prev_timestamp = None

    def quaternion_to_yaw(self, quaternion):
        x, y, z, w = quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def get_position_delta(self, linear_acceleration, dt):
        # 通过加速度积分两次得到位移
        dx = 0.5 * linear_acceleration[0] * dt * dt
        dy = 0.5 * linear_acceleration[1] * dt * dt
        return dx, dy

    def convert_to_habitat(self, quaternion, linear_acceleration, timestamp):
        """
        将ZED IMU数据转换为Habitat格式

        Args:
            quaternion: 四元数 [x,y,z,w]
            linear_acceleration: 线性加速度 [ax,ay,az]
            timestamp: 时间戳(微秒)

        Returns:
            dx: x方向位移变化
            dy: y方向位移变化 
            do: 朝向角度变化(弧度)
        """
        if self.prev_quaternion is None:
            self.prev_quaternion = quaternion
            self.prev_timestamp = timestamp
            return 0, 0, 0

        # 计算时间间隔(秒)
        dt = (timestamp - self.prev_timestamp) / 1e6

        # 计算朝向变化
        prev_yaw = self.quaternion_to_yaw(self.prev_quaternion)
        curr_yaw = self.quaternion_to_yaw(quaternion)
        do = curr_yaw - prev_yaw

        # 计算位置变化
        dx, dy = self.get_position_delta(linear_acceleration, dt)

        # 更新上一次的状态
        self.prev_quaternion = quaternion
        self.prev_timestamp = timestamp

        return dx, dy, do

# 使用示例
def main():
    converter = IMUConverter()

    # 在获取IMU数据时调用转换函数
    while True:
        # 假设从ZED获取数据
        quaternion = sensors_data.get_imu_data().get_pose().get_orientation().get()
        linear_acceleration = sensors_data.get_imu_data().get_linear_acceleration()
        timestamp = sensors_data.get_imu_data().timestamp.get_microseconds()

        # 转换为Habitat格式
        dx, dy, do = converter.convert_to_habitat(quaternion, linear_acceleration, timestamp)

        # 现在可以将dx, dy, do传给Habitat使用
        print(f"Delta: dx={dx:.3f}, dy={dy:.3f}, do={math.degrees(do):.1f}°")