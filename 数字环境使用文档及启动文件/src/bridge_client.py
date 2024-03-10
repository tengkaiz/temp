#!/usr/bin/python3
# -*- coding: utf-8 -*-

from time import sleep
from roslibpy import Topic, Ros, Time
import cv2
import keyboard
from threading import Thread
import os
import subprocess
import sys
import signal
import atexit
import platform
import yaml

Host     = "192.168.1.100"
Port     = 9090
Map_path = "map.png"
Linear   = 0.25
Angular  = 3.14
Dert     = 0.3

class FoxGlove2OriginCar:
    def __init__(self, host="10.168.1.197", port=9090, map_path="map.png",linear=0.5, angular=1.57, dert=0.1):
        self.host = host
        self.port = port
        self.image_path = map_path
        self.linear = linear
        self.angular = angular
        self.dert = dert
        self.argv = sys.argv
        self.system_str = platform.system()
        self.isStartTelemetry = False       # 是否开始遥测
        self.isOverTelemetry  = False       # 是否结束遥测
        self.signFlag         = False       # 检测是否更新上位机信号
        self.init()

    """
    @解释: 初始化
    """
    def init(self):
        self.init_cfg()
        self.init_arg()
        self.init_keymap()
        self.check_host()
        self.init_ros_interface()
        self.init_topic()
        self.open_foxglove()
        self.init_keyboard()
        self.keep()

    """
    @解释: 通过配置文件初始化
    """
    def init_cfg(self):
        try:
            with open('config.yaml', 'r', encoding='utf-8') as f:
                cfgDict: dict['str'] = yaml.load(f.read(), Loader=yaml.FullLoader)
            self.host = cfgDict['ip']
            self.port = int(cfgDict['port'])
            self.image_path = cfgDict['map_path']
            self.linear = float(cfgDict['linear'])
            self.angular = float(cfgDict['angular'])
            self.dert = float(cfgDict['dert'])
        except Exception as e:
            print("config.yaml文件不存在或格式有误,请检查!\n(若使用脚本运行则请忽略)")

    """
    @解释: 若用户在命令行传入传入参数,则初始化IP或者端口,这会覆盖所有ip和port
          命令行传参优先级最高
    """
    def init_arg(self):
        if len(self.argv) == 1:
            print("默认ip:{}.\n默认端口号:{}.".format(self.host, self.port))
        elif len(self.argv) == 2:
            self.host = self.argv[1]
            print("ip:{}.\n默认端口号:{}.".format(self.host, self.port))
        elif len(self.argv) == 3:
            self.host = self.argv[1]
            self.port = self.argv[2]
            print("ip:{}.\n端口号:{}.".format(self.host, self.port))
        else:
            print("************************")
            print("***** 传入参数错误 *****")
            print("************************")
            self.self.exit_wait()

    """
    @解释: 初始化速度发布消息,与键盘映射
    """
    def init_keymap(self):
        self.key_mapping = {
            'up'         : {'linear': {'x': self.linear,'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z': 0.0          }},
            'down'       : {'linear': {'x':-self.linear,'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z': 0.0          }},
            'left'       : {'linear': {'x': 0.0,        'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z': self.angular }},
            'right'      : {'linear': {'x': 0.0,        'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z':-self.angular }},
            'up_left'    : {'linear': {'x': self.linear,'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z': self.angular }},
            'up_right'   : {'linear': {'x': self.linear,'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z':-self.angular }},
            'down_left'  : {'linear': {'x':-self.linear,'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z': self.angular }},
            'down_right' : {'linear': {'x':-self.linear,'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z':-self.angular }},
            'stop'       : {'linear': {'x': 0.0,        'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z': 0.0          }}
        }

    """
    @解释: 检查网络是否可达
          若可达,则初始化ros服务
          若不可达,则退出
    """
    def check_host(self):
        if self.system_str == 'Windows':
            ret = os.system("ping -n 1 -w 1 {}".format(self.host))
        elif self.system_str == 'Linux':
            ret = os.system("ping -c 1 -w 1 {}".format(self.host))
        spa = ' '
        l = len(self.host)
        spa *= 13 - l
        print('\n\n')
        if ret:
            print("************************")
            print("*** ip:{}{} ***".format(self.host, spa))
            print("*** 无法访问, 请重试 ***")
            print("************************")
            self.exit_wait()
        else:
            print("************************")
            print("*** ip:{}{} ***".format(self.host, spa))
            print("*** ip有效, 正在连接 ***")
            print("************************")
            print("\n连接主机 ws://{}:{}.".format(self.host, self.port))
            self.ros = Ros(host=self.host, port=self.port)

    """
    @解释: 尝试建立连接
    """
    def init_ros_interface(self):
        try:
            self.ros.run()
            sleep(1)
            if self.ros.is_connected:
                print("连接已建立 ws://{}:{}.".format(self.host, self.port))
        except Exception as e:
            print("连接失败:\n1. 请检查端口是否开启\n2. 请检查端口号是否正确\n3. 请检查主机是否开启了rosbridge ws://{}:{}.".format(self.host, self.port))
            self.exit_wait()

    """
    @解释: 初始化Topic
    """
    def init_topic(self):
        # 地图发布相关设置
        self.map_pub  = Topic(self.ros, "/map", 'nav_msgs/OccupancyGrid',latch=True)
        self.vel_pub  = Topic(self.ros, '/cmd_vel', 'geometry_msgs/Twist')
        self.sign_pub = Topic(self.ros, '/sign_foxglove', 'std_msgs/msg/Int32',latch=True)
        self.sign_sub = Topic(self.ros, '/sign4return', 'std_msgs/msg/Int32')
        self.sign_sub.subscribe(self.sign_sub_callback)
        self.sign = {'data': 0}

        if os.path.exists(self.image_path):
            print("存在地图, 正在解析...")
            image = cv2.imread(self.image_path, cv2.IMREAD_GRAYSCALE)
            image = cv2.flip(image, -1)  # 水平和垂直翻转
            image = cv2.flip(image, 1)
        else:
            print("当前程序目录:不存在 map.png 请检查地图...")
            self.exit_wait()
        map_width, map_height = image.shape[1], image.shape[0]
        occupancy_data = []
        for row in image:
            occupancy_data.extend([0 if pixel < 128 else -1 for pixel in row])
        self.map_data = {
            'header': {
                'stamp': {
                    'sec': Time.now().secs,
                    'nanosec': Time.now().nsecs
                },
                'frame_id': 'odom_combined'
            },
            'info': {
                'map_load_time': {
                    'sec': 0,
                    'nanosec': 0
                },
                'resolution': 5.0 / map_width,  # 5 meters corresponds to the map width
                'width': map_width,
                'height': map_height,
                'origin': {
                    'position': {
                        'x': 0,
                        'y': 0,
                        'z': 0
                    },
                    'orientation': {
                        'x': 0,
                        'y': 0,
                        'z': 0,
                        'w': 1
                    }
                }
            },
            'data': occupancy_data
        }
        self.map_pub.publish(self.map_data)
        self.sign_pub.publish(self.sign)
        print("信号连接正常.")
        print("已发布地图.")
        self.help_tip()
        print("后台正在持续监听键盘命令.", end='  ')

    """
    @解释: 初始化键盘监听线程
    """
    def init_keyboard(self):
        self.keyboard_thread = Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    """
    @解释: 打开上位机,并且获取进程以控制关闭
    """
    def open_foxglove(self):
        if self.system_str == "Windows":
            print("当前操作系统为: Windows.")
            local_appdata_path = os.getenv("LOCALAPPDATA")
            if local_appdata_path:
                foxglove_path = os.path.join(local_appdata_path, "Programs", "foxglove-studio", "Foxglove Studio.exe")
            else:
                print("未找到环境变量 LOCALAPPDATA.")
            try:
                # subprocess.Popen(foxglove_path) # 终端会显示上位机软件的输出, 若要查看foxglove输出，请取消这行注释, 并注释下一行
                foxglove_process = subprocess.Popen(foxglove_path, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print("打开Foxglove软件！")
                # 获取进程ID
                foxglove_pid = foxglove_process.pid
                print("在关闭终端时将会关闭Foxglove软件...")

                def on_exit():
                    try:
                        # 发送 SIGTERM 信号终止进程
                        os.kill(foxglove_pid, signal.SIGTERM)
                        print("已关闭Foxglove软件！")
                    except OSError:
                        pass

                # 注册终端关闭时的事件处理函数
                atexit.register(on_exit)

            except FileNotFoundError:
                print("无法找到指定的Foxglove软件, 请检查路径是否正确.")
            except Exception as e:
                print("发生了错误:", e)
        elif self.system_str == "Linux":
            print("当前操作系统为:Linux,需要手动以用户模式打开Foxglove.")
            # foxglove_path = "foxglove-studio"
            # ret = os.system(foxglove_path)
            # print(ret)

    """
    @解释: 保持循环,等待结束
    """
    def keep(self):
        try:
            while True:
                for i in range(4):
                    sleep(0.5)
                    print('\b\b', ['/', '|', '\\', '-'][i], end="", flush=True)
        except KeyboardInterrupt:
            print("\n检测到ctrl + c, 是否确认退出！")
            self.exit_wait()
            self.ros.terminate()

    """
    @解释: 退出
    """
    def exit_wait(self):
        print("按ESC退出.")
        while not keyboard.is_pressed('ESC'):
            sleep(0.1)
        exit()

    """
    @解释: 更新并设置速度
    """
    def update_vel(self):
        self.key_mapping['up']['linear']['x']          =  self.linear
        self.key_mapping['down']['linear']['x']        = -self.linear
        self.key_mapping['left']['angular']['z']       =  self.angular
        self.key_mapping['right']['angular']['z']      = -self.angular
        self.key_mapping['up_left']['linear']['x']     =  self.linear
        self.key_mapping['up_left']['angular']['z']    =  self.angular
        self.key_mapping['up_right']['linear']['x']    =  self.linear
        self.key_mapping['up_right']['angular']['z']   = -self.angular
        self.key_mapping['down_left']['linear']['x']   = -self.linear
        self.key_mapping['down_left']['angular']['z']  =  self.angular
        self.key_mapping['down_right']['linear']['x']  = -self.linear
        self.key_mapping['down_right']['angular']['z'] = -self.angular

    """
    @解释: 键盘监听
    """
    def keyboard_listener(self):
        print("\n\n")
        print("------------------------------------")
        print("**** 按下   [ r ]   开始控制小车 ****")
        print("**** 或者等待上位机发送开始遥测信号 ****")
        print("------------------------------------")
        while True:
            sleep(0.05)
            if keyboard.is_pressed('r') or self.isStartTelemetry:         # 激活键盘控制
                self.isOverTelemetry = False
                self.isStartTelemetry = False
                print("\n正在继续键盘监听.")
                break
        while True:
            sleep(0.05)
            # 优先回复上位机信号
            self.update_sign()
            if keyboard.is_pressed('p') or self.isOverTelemetry:          # 退出激活键盘控制
                self.isOverTelemetry = False
                self.isStartTelemetry = False
                print("\n已退出键盘监听.")
                print("同时按下[ r ]继续键盘监听.")
                while True:
                    if keyboard.is_pressed('r') or self.isStartTelemetry:         # 激活键盘控制
                        self.isOverTelemetry = False
                        self.isStartTelemetry = False
                        print("\n正在继续键盘监听.")
                        self.sign_pub.publish(self.sign)
                        break
                    sleep(0.05)
            if keyboard.is_pressed('w'):                                  # 前进
                if keyboard.is_pressed('a'):                              # 前进加左转
                    self.vel_pub.publish(self.key_mapping['up_left'])
                elif keyboard.is_pressed('d'):                            # 前进加右转
                    self.vel_pub.publish(self.key_mapping['up_right'])
                else:
                    self.vel_pub.publish(self.key_mapping['up'])
            elif keyboard.is_pressed('s'):                                # 后退
                if keyboard.is_pressed('d'):                              # 前进加左转
                    self.vel_pub.publish(self.key_mapping['down_left'])
                elif keyboard.is_pressed('a'):                            # 前进加右转
                    self.vel_pub.publish(self.key_mapping['down_right'])
                else:
                    self.vel_pub.publish(self.key_mapping['down'])
            elif keyboard.is_pressed('a'):                                # 仅左转
                self.vel_pub.publish(self.key_mapping['left'])
            elif keyboard.is_pressed('d'):                                # 仅右转
                self.vel_pub.publish(self.key_mapping['right'])

            elif keyboard.is_pressed("up"):
                self.linear += self.dert
                print("线速度设置为:{:.2f}m/s.".format(self.linear))
                self.update_vel()
                sleep(0.2)
            elif keyboard.is_pressed("down"):
                if self.linear - self.dert < 0:
                    print("速度设置失败！(线速度为:{:.2f}).".format(self.linear))
                else:
                    self.linear -= self.dert
                    print("线速度设置为:{:.2f}m/s.".format(self.linear))
                self.update_vel()
                sleep(0.2)
            elif keyboard.is_pressed("left"):
                if self.angular - self.dert < 0:
                    print("角度设置失败！(转角为:{:.2f}rad).".format(self.angular))
                else:
                    self.angular -= self.dert
                    print("角度设置为:{:.2f}rad.".format(self.angular))
                self.update_vel()
                sleep(0.2)
            elif keyboard.is_pressed("right"):
                self.angular += self.dert
                print("转角设置为:{:.2f}rad.".format(self.angular))
                self.update_vel()
                sleep(0.2)

            elif keyboard.is_pressed('t'):
                self.help_tip()
                sleep(0.5)
            
            elif keyboard.is_pressed('m'):
                self.map_pub.publish(self.map_data)
            else:
                self.vel_pub.publish(self.key_mapping['stop'])


    def update_sign(self):
        if self.signFlag:
            self.sign_pub.publish(self.signMsg)
            self.signFlag = False

    """
    @解释: 提示
    """
    def help_tip(self):
        print("\n\n提示：")
        print("按   [ p ] 退出键盘控制.")
        print("按   [ r ] 回到键盘控制.")
        print("按   [ m ] 重新发布地图.")
        print("按   [ t ] 显示按键帮助.", end="\n\n")

        print("控制：")
        print("--- [ w ] --- 前进: {:.2f} m/s.".format(self.linear))
        print("--- [ a ] --- 左转: {:.2f} rad.".format(self.angular))
        print("--- [ d ] --- 右转: {:.2f} rad.".format(-self.angular))
        print("--- [ s ] --- 后退: {:.2f} m/s.".format(-self.linear))

        print("调整速度(使用键盘的方向键).")
        print("---  [   up  ]  --- 增加线速度.")
        print("---  [  left ]  --- 减小转角.")
        print("---  [ right ]  --- 增加转角.")
        print("---  [  down ]  --- 减小线速度.")

    def sign_sub_callback(self, msg):
        if msg['data'] == 5:
            print("\n接收到[正在C区进行遥测]信号")
            self.isStartTelemetry = True
            
        elif msg['data'] == 6:
            print("\n接收到[C区出口结束遥测]信号")
            self.isOverTelemetry  = True
        self.signMsg = msg
        self.signFlag = True


if __name__ == "__main__":
    foxGlove2OriginCar = FoxGlove2OriginCar(
        host     = Host,
        port     = Port,
        map_path = Map_path,
        linear   = Linear,
        angular  = Angular,
        dert     = Dert
    )
