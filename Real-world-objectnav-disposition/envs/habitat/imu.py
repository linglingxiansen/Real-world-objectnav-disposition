import pyzed.sl as sl
import cv2
import numpy as np
import math
import time 
## 
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
        print(self.prev_quaternion,quaternion)
        self.prev_quaternion = quaternion
        self.prev_timestamp = timestamp
        
        return dx, dy, do
# Basic class to handle the timestamp of the different sensors to know if it is a new sensors_data or an old one
class TimestampHandler:
    def __init__(self):
        self.t_imu = sl.Timestamp()
        self.t_baro = sl.Timestamp()
        self.t_mag = sl.Timestamp()

    ##
    # check if the new timestamp is higher than the reference one, and if yes, save the current as reference
    def is_new(self, sensor):
        if (isinstance(sensor, sl.IMUData)):
            new_ = (sensor.timestamp.get_microseconds() > self.t_imu.get_microseconds())
            if new_:
                self.t_imu = sensor.timestamp
            return new_
        elif (isinstance(sensor, sl.MagnetometerData)):
            new_ = (sensor.timestamp.get_microseconds() > self.t_mag.get_microseconds())
            if new_:
                self.t_mag = sensor.timestamp
            return new_
        elif (isinstance(sensor, sl.BarometerData)):
            new_ = (sensor.timestamp.get_microseconds() > self.t_baro.get_microseconds())
            if new_:
                self.t_baro = sensor.timestamp
            return new_

##
#  Function to display sensor parameters
def printSensorParameters(sensor_parameters):
    if sensor_parameters.is_available:
        print("*****************************")
        print("Sensor type: " + str(sensor_parameters.sensor_type))
        print("Max rate: "  + str(sensor_parameters.sampling_rate) + " "  + str(sl.SENSORS_UNIT.HERTZ))
        print("Range: "  + str(sensor_parameters.sensor_range) + " "  + str(sensor_parameters.sensor_unit))
        print("Resolution: " + str(sensor_parameters.resolution) + " "  + str(sensor_parameters.sensor_unit))
        if not math.isnan(sensor_parameters.noise_density):
            print("Noise Density: "  + str(sensor_parameters.noise_density) + " " + str(sensor_parameters.sensor_unit) + "/√Hz")
        if not math.isnan(sensor_parameters.random_walk):
            print("Random Walk: "  + str(sensor_parameters.random_walk) + " " + str(sensor_parameters.sensor_unit) + "/s/√Hz")
    


def main():
    converter = IMUConverter()
    # Create a Camera object
    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NONE

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS :
        print(repr(err))
        zed.close()
        exit(1)

    # Get camera information sensors_data
    info = zed.get_camera_information()

    cam_model = info.camera_model
    if cam_model == sl.MODEL.ZED :
        print("This tutorial only supports ZED-M and ZED2 camera models, ZED does not have additional sensors")
        exit(1)

    # Display camera information (model,S/N, fw version)
    print("Camera Model: " + str(cam_model))
    print("Serial Number: " + str(info.serial_number))
    print("Camera Firmware: " + str(info.camera_configuration.firmware_version))
    print("Sensors Firmware: " + str(info.sensors_configuration.firmware_version))

    # Display sensors parameters (imu,barometer,magnetometer)
    printSensorParameters(info.sensors_configuration.accelerometer_parameters) # accelerometer configuration
    printSensorParameters(info.sensors_configuration.gyroscope_parameters) # gyroscope configuration
    printSensorParameters(info.sensors_configuration.magnetometer_parameters) # magnetometer configuration
    printSensorParameters(info.sensors_configuration.barometer_parameters) # barometer configuration
    
    # Used to store the sensors timestamp to know if the sensors_data is a new one or not
    ts_handler = TimestampHandler()

    # Get Sensor Data for 5 seconds
    sensors_data = sl.SensorsData()
    time_0 = time.time() 
    while time.time()-time_0 < 500 :
        # retrieve the current sensors sensors_data
        # Depending on your Camera model or its firmware, differents sensors are presents.
        # They do not run at the same rate: Therefore, to do not miss samples we iterate as fast as we can and compare timestamp to know when a sensors_data is a new one
        # NOTE: There is no need to acquire images with grab() function. Sensors sensors_data are running in a separated internal capture thread.
        if zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS :
            # Check if the data has been updated since the last time
            # IMU is the sensor with the highest rate
            if ts_handler.is_new(sensors_data.get_imu_data()):

                print(" - IMU:")
                # Filtered orientation quaternion
                quaternion = sensors_data.get_imu_data().get_pose().get_orientation().get()
                print(" \t Orientation: [ Ox: {0}, Oy: {1}, Oz {2}, Ow: {3} ]".format(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
                
                # linear acceleration
                linear_acceleration = sensors_data.get_imu_data().get_linear_acceleration()
                print(" \t Acceleration: [ {0} {1} {2} ] [m/sec^2]".format(linear_acceleration[0], linear_acceleration[1], linear_acceleration[2]))

                # angular velocities
                angular_velocity = sensors_data.get_imu_data().get_angular_velocity()
                print(" \t Angular Velocities: [ {0} {1} {2} ] [deg/sec]".format(angular_velocity[0], angular_velocity[1], angular_velocity[2]))

                # Check if Magnetometer data has been updated (not the same frequency than IMU)
                if ts_handler.is_new(sensors_data.get_magnetometer_data()):
                    magnetic_field_calibrated = sensors_data.get_magnetometer_data().get_magnetic_field_calibrated()
                    print(" - Magnetometer\n \t Magnetic Field: [ {0} {1} {2} ] [uT]".format(magnetic_field_calibrated[0], magnetic_field_calibrated[1], magnetic_field_calibrated[2]))
                
                # Check if Barometer data has been updated 
                if ts_handler.is_new(sensors_data.get_barometer_data()):
                    magnetic_field_calibrated = sensors_data.get_barometer_data().pressure
                    print(" - Barometer\n \t Atmospheric pressure: {0} [hPa]".format(sensors_data.get_barometer_data().pressure))
        
                dx, dy, do = converter.convert_to_habitat(quaternion, linear_acceleration, time.time())

                print(dx,dy,do)
        time.sleep(1)

    zed.close()
    return 0

if __name__ == "__main__":
    main()