import socket
import time
import json
host, port = '192.168.10.10', 31001 # host为底盘ip，port为端口

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((host, port))
print("Connected to the server.")

# 前后左右
points = {
    "forward": '/api/joy_control?angular_velocity=0&linear_velocity=0.5',
    "back": '/api/joy_control?angular_velocity=0.0&linear_velocity=-0.25',
    "left": '/api/joy_control?angular_velocity=1.5 &linear_velocity=0.0',
    "right": '/api/joy_control?angular_velocity=-1.5&linear_velocity=0.0'
} # 线速度、角速度可自行更改
direction = "left"
status = '/api/robot_status'

client.send(status.encode("utf-8"))
data = client.recv(1024).decode()
print(data)

data_json = json.loads(data)
results = data_json['results']
print(data_json['results'])
print(results['current_pose'])
pose = results['current_pose']
print(pose['theta'])

for i in range(3):
    client.send(points[direction].encode("utf-8"))
    data = client.recv(1024).decode()
    time.sleep(1)

# time.sleep(1)
# point = '/api/move?location=' + '1.68,4.00,2.3169' # location为地图上的x,y,theta
# print(point)
# client.send(point.encode("utf-8"))
# data = client.recv(1024).decode()
# print(data)