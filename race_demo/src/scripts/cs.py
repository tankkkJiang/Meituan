#!/usr/bin/env python3
import rospy
import json
import numpy as np
from enum import Enum
import multiprocessing
import math
import itertools
import threading

from std_msgs.msg import String
from race_demo.msg import BillStatus
from race_demo.msg import BindCargo
from race_demo.msg import CarPhysicalStatus
from race_demo.msg import CarRouteInfo
from race_demo.msg import DroneMsg
from race_demo.msg import DronePhysicalStatus
from race_demo.msg import DroneWayPoint
from race_demo.msg import DroneWayPointInfo
from race_demo.msg import DynamicPosition
from race_demo.msg import EulerAngle
from race_demo.msg import EventMsg
from race_demo.msg import PanoramicInfo
from race_demo.msg import Position
from race_demo.msg import UnBindInfo
from race_demo.msg import UserCmdRequest
from race_demo.msg import UserCmdResponse
from race_demo.msg import UserPhysicalStatus
from race_demo.msg import Voxel
from race_demo.srv import QueryVoxel, QueryVoxelRequest

from a_star import navigate_with_astar
import pymtmap

# demo定义的状态流转

Moving_car_cycle = 32
Preparation_Cycle = 20
move_car_time = 12

class WorkState(Enum):
    START = 1
    TEST_MAP_QUERY = 2
    MOVE_CAR_GO_TO_LOADING_POINT = 3
    MOVE_DRONE_ON_CAR = 4
    MOVE_CARGO_IN_DRONE = 5
    MOVE_CAR_TO_LEAVING_POINT = 6
    RELEASE_DRONE_OUT = 7
    RELEASE_CARGO = 8
    RELEASE_DRONE_RETURN = 9
    MOVE_CAR_BACK_TO_LOADING_POINT = 10
    DRONE_BATTERY_REPLACEMENT = 11
    DRONE_RETRIEVE = 12
    FINISHED = 13
    SELACT_WAYBILL_CAR_DRONE =14
    DRONE_LANDING = 15

class Car:
    def __init__(self, car_sn, target_index, points):
        self.points = points
        self.car_sn = car_sn
        self.target_index = target_index
        self.target_pos = points[target_index]
        # print(f"车辆 {self.car_sn}，目标索引: {self.target_index}，目标位置: {self.target_pos}")

    def set_target(self):
        """设置移动目标"""
        self.target_index = (self.target_index + 1) % len(self.points)  # 逆时针选择下一个点
        self.target_pos = self.points[self.target_index]
        # print(f"车辆 {self.car_sn} 已更新，目标索引: {self.target_index}，目标位置: {self.target_pos}")



class DemoPipeline:
    def __init__(self):
        # 初始化ros全局变量
        self.state = WorkState.START
        rospy.init_node('race_demo')
        self.cmd_pub = rospy.Publisher(
            '/cmd_exec', UserCmdRequest, queue_size=10000)
        self.info_sub = rospy.Subscriber(
            '/panoramic_info',
            PanoramicInfo,
            self.panoramic_info_callback,
            queue_size=100)
        self.map_client = rospy.ServiceProxy('query_voxel', QueryVoxel)
        # 读取配置文件和信息
        self.running_start_time = 0
        self.running_start_time_ms = 0
        print(f"开始的毫秒时间戳 - {self.running_start_time_ms}")
        with open('/config/config.json', 'r') as file:
            self.config = json.load(file)
        self.drone_infos = self.config['taskParam']['droneParamList']
        self.car_infos = self.config['taskParam']['magvParamList']
        self.loading_cargo_point = self.config['taskParam']['loadingCargoPoint']
        self.map_boundary = self.config['taskParam']['mapBoundaryInfo']

        self.waybill_infos = self.config['taskParam']['waybillParamList']
        print(self.waybill_infos)
        # 在派发前按 betterTime + timeout 排序waybills
        # self.waybill_infos.sort(key=lambda x: x['betterTime'] + x['timeout'])
        self.waybill_infos.sort(key=lambda x: x['orderTime'])
        # self.waybill_infos.sort(key=lambda x: x['orderTime'] + x['timeout'])

        self.unloading_cargo_stations = self.config['taskParam']['unloadingCargoStationList']
        self.drone_sn_list = [drone['droneSn'] for drone in self.drone_infos]
        self.car_sn_list = [car['magvSn'] for car in self.car_infos]
        self.peer_id = self.config['peerId']
        self.task_guid = self.config['taskParam']['guid']
        self.car_physical_status = None
        self.drone_physical_status = None
        self.bills_status = None
        self.score = None
        self.events = None
        self.waybill_start_time_millis = None
        self.order_semaphore = threading.Semaphore(1)  # 初始不可用
        self.drone_takeoff_semaphore = threading.Semaphore(0)  # 初始化信号量为 0，表示当前不可用
        self.drone_landing_semaphore = threading.Semaphore(1)  # 初始化为 1，表示初始时允许小车移动
        self.is_landing_blocked = False  # 用于避免重复获取降落信号量的标志位
        self.lock = threading.Lock()                     # 用于保护共享资源的锁
        self.loss_waybill = 0
        self.giveup_waybill = 0

    # 仿真回调函数，获取实时信息
    def panoramic_info_callback(self, panoramic_info):
        self.car_physical_status = panoramic_info.cars
        self.drone_physical_status = panoramic_info.drones
        self.bills_status = panoramic_info.bills
        # print(f"self bills status:{self.bills_status}")
        self.score = panoramic_info.score
        self.events = panoramic_info.events

    # 系统初始化(按需)
    def sys_init(self):
        self.state = WorkState.TEST_MAP_QUERY

    # 测试地图查询接口，可用这个或地图SDK进行航线规划
    def test_map_query(self):
        request = QueryVoxelRequest()
        request.x = 1.0
        request.y = 2.0
        request.z = -3.0
        response = self.map_client(request)
        print(response)
        if response.success:
            self.state = WorkState.SELACT_WAYBILL_CAR_DRONE
    
    # 检测位置到达的函数
    def des_pos_reached(self, des_pos, cur_pos, threshold):
        des = np.array([des_pos.x, des_pos.y, des_pos.z])
        cur = np.array([cur_pos.x, cur_pos.y, cur_pos.z])
        return np.linalg.norm(np.array(des - cur)) < threshold

    # 移动地面车辆的函数
    def move_car_with_start_and_end(self, car_sn, start, end, time_est, next_state):
        print(f"{car_sn}与环境信息交流，开始移动")

        # 创建UserCmdRequest消息对象
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = car_sn
        msg.car_route_info.way_point.append(start)
        msg.car_route_info.way_point.append(end)
        msg.car_route_info.yaw = 0.0

        # 发送初始移动命令
        self.cmd_pub.publish(msg)
        start_time = rospy.Time.now()  # 获取当前时间

        while True:
            # 获取当前小车状态
            car_physical_status = next((cps for cps in self.car_physical_status if cps.sn == car_sn), None)
            if car_physical_status is None:
                # print(f"未找到车辆 {car_sn} 的物理状态信息，等待重试...")
                rospy.sleep(1)
                continue

            car_pos = car_physical_status.pos.position

            # 检查小车是否已经确定离开出发点
            if not self.des_pos_reached(car_pos, start, 1):
                rospy.sleep(1)
                break

            # 如果小车在2秒内没有离开出发点，则重新发送移动命令
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            if elapsed_time > 2 and self.des_pos_reached(car_pos, start, 0.5):
                # print(f"{car_sn}未在2秒内离开出发点，重新发送移动命令")
                self.cmd_pub.publish(msg)
                start_time = rospy.Time.now()  # 重置开始时间

            rospy.sleep(0.5)  # 每0.5秒检查一次


    # 往车上挪机
    def move_drone_on_car(self, car_sn, drone_sn, time_est, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_DRONE_ON_CAR
        msg.binding_drone.car_sn = car_sn
        msg.binding_drone.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        state = next_state

    # 往飞机上挂餐
    def move_cargo_in_drone(self, cargo_id, drone_sn, time_est):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_CARGO_IN_DRONE
        msg.binding_cargo.cargo_id = cargo_id
        msg.binding_cargo.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        

    # 飞机航线飞行函数
    def fly_one_route(self, drone_sn, route, speed, time_est, next_state, is_departure):
        print(f"drone_sn:{drone_sn}:无人机开始起飞")
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_EXEC_ROUTE
        msg.drone_way_point_info.droneSn = drone_sn
        takeoff_point = DroneWayPoint()
        takeoff_point.type = DroneWayPoint.POINT_TAKEOFF
        takeoff_point.timeoutsec = 1000
        msg.drone_way_point_info.way_point.append(takeoff_point)
        for waypoint in route:
            middle_point = DroneWayPoint()
            middle_point.type = DroneWayPoint.POINT_FLYING
            middle_point.pos.x = waypoint.x
            middle_point.pos.y = waypoint.y
            middle_point.pos.z = waypoint.z
            middle_point.v = speed
            middle_point.timeoutsec = 1000
            msg.drone_way_point_info.way_point.append(middle_point)
        land_point = DroneWayPoint()
        land_point.type = DroneWayPoint.POINT_LANDING
        land_point.timeoutsec = 1000
        msg.drone_way_point_info.way_point.append(land_point)
        self.cmd_pub.publish(msg)
        rospy.sleep(1)
        # 仅在出发状态时释放信号量，通知后续车辆可以移动
        if is_departure:
            self.drone_takeoff_semaphore.release()
            print(f"drone_sn:{drone_sn}:出发的无人机已起飞，释放起飞锁，允许后续车辆开始移动。")

        rospy.sleep(time_est)
        # state = next_state

    # 抛餐函数
    def release_cargo(self, drone_sn, time_est, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_RELEASE_CARGO
        msg.drone_msg.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        state = next_state

    # 换电函数
    def battery_replacement(self, drone_sn, time_est, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_BATTERY_REPLACEMENT
        msg.drone_msg.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        state = next_state

    # Function to calculate the Euclidean distance between two points
    def calculate_distance(self, point1, point2):
        return math.sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2 + (point2.z - point1.z)**2)
        
    # 回收飞机函数
    def drone_retrieve(self, drone_sn, car_sn, time_est, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_DRONE_ON_BIRTHPLACE
        msg.unbind_info.drone_sn = drone_sn
        msg.unbind_info.car_sn = car_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        state = next_state

    # 查询初始化位置
    def query_car_init_pos(self, car_sn):
        car_physical_status = next(
            (car for car in self.car_physical_status if car.sn == car_sn), None)
        car_init_pos = car_physical_status.pos.position
        # print(f"{car_sn}初始位置为{car_init_pos}")
        return car_init_pos
    
    def init_car(self, car):
        car_physical_status = next(
            (cps for cps in self.car_physical_status if cps.sn == car.car_sn), None)
        car_pos = car_physical_status.pos.position
        self.move_car_to_new_init_pos(car.car_sn, car_pos, car.target_pos)

    # 移动小车到初始化位置
    def move_car_to_new_init_pos(self, car_sn, start, end):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = car_sn

        # 中间的直走
        if car_sn == "SIM-MAGV-0001":
            print("移动1号车")
            msg.car_route_info.way_point.append(start)
            msg.car_route_info.way_point.append(end)
        # 最外侧的两个绕路
        elif car_sn == "SIM-MAGV-0002":
            print("移动2号车")
            middle_1 = Position(184, 437, -16)
            middle_2 = Position(184, 431, -16)
            msg.car_route_info.way_point.append(start)
            msg.car_route_info.way_point.append(middle_1)
            msg.car_route_info.way_point.append(middle_2)
            msg.car_route_info.way_point.append(end)

        elif car_sn == "SIM-MAGV-0005":
            print("移动5号车")
            middle_1 = Position(190, 438, -16)
            middle_2 = Position(183, 438, -16)
            msg.car_route_info.way_point.append(start)
            msg.car_route_info.way_point.append(middle_1)
            msg.car_route_info.way_point.append(middle_2)
            msg.car_route_info.way_point.append(end)

        # 中间的两边也要绕一下
        elif car_sn == "SIM-MAGV-0003":
            print("移动3号车")
            middle_1 = Position(189, 446, -16)
            middle_2 = Position(189, 439, -16)
            msg.car_route_info.way_point.append(start)
            msg.car_route_info.way_point.append(middle_1)
            msg.car_route_info.way_point.append(middle_2)
            msg.car_route_info.way_point.append(end)

        elif car_sn == "SIM-MAGV-0006":
            print("移动6号车")
            middle_1 = Position(193, 446, -16)
            middle_2 = Position(193, 439, -16)
            msg.car_route_info.way_point.append(start)
            msg.car_route_info.way_point.append(middle_1)
            msg.car_route_info.way_point.append(middle_2)
            msg.car_route_info.way_point.append(end)
        
        elif car_sn == "SIM-MAGV-0004":
            print("移动4号车")
            middle = Position(197, 431, -16)
            msg.car_route_info.way_point.append(start)
            msg.car_route_info.way_point.append(middle)
            msg.car_route_info.way_point.append(end)

        msg.car_route_info.yaw = 0.0
        self.cmd_pub.publish(msg)

    # 移动单辆小车
    def move_car(self, car):
        # print(f"car{car.car_sn}移动单辆小车")
        car.set_target()
        car_physical_status = next(
            (cps for cps in self.car_physical_status if cps.sn == car.car_sn), None)
        # 检查小车状态是否有效
        if car_physical_status:
            car_pos = car_physical_status.pos.position
            # print(f"car{car.car_sn}, 现在的位置: {car_pos}, 移动目标位置: {car.target_pos}")
            self.move_car_with_start_and_end(car.car_sn, car_pos, car.target_pos, move_car_time, WorkState.SELACT_WAYBILL_CAR_DRONE)
        else:
            print(f"car{car.car_sn}未找到物理状态信息")
            # 更新小车当前位置和目标位置
    
    # 小车按照循环点移动
    def move_car_to_target_pos(self, car_list):
        print("正在调用循环小车移动")
        threads = []
        # print(f"car list:{car_list}")
        processed_cars = set()  # 使用集合跟踪已经创建线程的小车
        # 创建所有线程
        for car in car_list:
            if car.car_sn not in processed_cars:  # 检查小车是否已经创建线程
                # print(f"car{car.car_sn}开始创建线程")
                processed_cars.add(car.car_sn)  # 将小车加入已处理集合
                thread = threading.Thread(
                    target=self.move_car, args=(car,)
                )
                threads.append(thread)
        for thread in threads:  
            thread.start()
        # 等待所有线程完成
        for thread in threads:
            thread.join()
        print("小车位置移动完成")

    def waybill_classification(self):  # 这里的订单分类只分最优的4个卸货点
        waybill_points = [
            {'x': 146, 'y': 186, 'z': -34},
            {'x': 508, 'y': 514, 'z': -22},
            {'x': 490, 'y': 390, 'z': -22},
            {'x': 430, 'y': 184, 'z': -10},
            {'x': 528, 'y': 172, 'z': -20},
            {'x': 564, 'y': 394, 'z': -16}
        ] 
        def calculate_distance(p1, p2):
            return math.sqrt((p1['x'] - p2['x']) ** 2 + (p1['y'] - p2['y']) ** 2)       
        groups = [[] for _ in range(6)]
        for i in range(len(self.waybill_infos)):
            waybill = self.waybill_infos[i]
            target_position = waybill['targetPosition']
            for index, point in enumerate(waybill_points):
                distance = calculate_distance(target_position, point)
                if distance < 1:
                    groups[index].append(waybill)
                    break 

        # 在每个组内按orderTime进行排序
        for group in groups:
            group.sort(key=lambda x: x['orderTime'])
        # 现在根据每个组中第一个条目的orderTime对所有组进行排序，如果组不为空
        sorted_groups = sorted(groups, key=lambda g: g[0]['orderTime'] if g else float('inf'))

        return sorted_groups

    # 订单分组
    def group_waybills(self, waybill_infos, takeoff_point):
        # 创建六个区域的空列表
        groups = [[] for _ in range(36)]
        center_x, center_y = takeoff_point.x, takeoff_point.y

        for waybill in waybill_infos:
            pos_x, pos_y = waybill['targetPosition']['x'], waybill['targetPosition']['y']
            # 计算相对角度
            angle = math.atan2(pos_y - center_y, pos_x - center_x)  # 计算角度（弧度）
            angle_degrees = math.degrees(angle)  # 转换为度数

            # 处理负角度，确保范围在 0 到 360 度
            if angle_degrees < 0:
                angle_degrees += 360

            # 确定区域索引（每个区域60度）
            region_index = int(angle_degrees // 10)  # 0-5区间
            groups[region_index].append(waybill)

        return groups
    
    # 调度小车和无人机完成订单
    def dispatching(self, car_list, loading_pos, birth_pos, takeoff_pos, landing_pos, waybill, flying_height, state, is_empty_car, bind_cargo_attempts):       
        flag = True
        waybill_start_time = rospy.Time.now()
        print(f"订单{waybill['index']}: Begin to dispatch, 还未进入选车机")      
        while not rospy.is_shutdown():
            if state == WorkState.SELACT_WAYBILL_CAR_DRONE:
                if self.waybill_count_start == 0:
                    # 利用第一单获取准确的启动时间戳，存入共享self.running_start_time_ms中。
                    order_status = next(
                        (order for order in self.bills_status if order.index == waybill['index']), None)
                    # 打印订单状态检查
                    if order_status is not None:
                        # print(f"初始化订单: Found order status: {order_status}")
                        better_time_ms = order_status.betterTime
                        # print(f"初始化订单: betterTime for order index: {better_time_ms}")
                        self.running_start_time_ms = better_time_ms - waybill['betterTime']
                        print(f"设置启动时间戳 Running start time (ms): {self.running_start_time_ms}")
                    else:
                        print("Order with index not found.")

                print(f"订单{waybill['index']}正在等待前一单移车完成/放弃执行再开始订单...")
                self.order_semaphore.acquire()  # (-1)阻塞，等待前一单完成并释放信号量

                with self.lock:
                    self.waybill_count_start += 1

                start_to_dispatch_time = (rospy.Time.now() - waybill_start_time).to_sec()
                print(f"已开始的订单数{self.waybill_count_start}, 丢弃订单数{self.giveup_waybill}, 失败订单数{self.loss_waybill}, 当前订单{waybill['index']}的小车无人机开始进行初始化，从提取订单到初始化等待了{start_to_dispatch_time}秒")
                dispatching_start_time = rospy.Time.now()

                while True:
                    car_physical_status = next(
                        (car for car in self.car_physical_status if self.des_pos_reached(car.pos.position, loading_pos, 1) and car.car_work_state == CarPhysicalStatus.CAR_READY), None)
                    if car_physical_status is not None:
                        print(f"订单{waybill['index']}找到小车")
                        break
                    rospy.sleep(0.1)
                car_sn = car_physical_status.sn 
                drone_sn = car_physical_status.drone_sn
                # 挑选无人机
                if drone_sn == '':
                    print(f"订单{waybill['index']}, {car_sn}挑选无人机，当前小车没有无人机")
                    # 遍历无人机列表，挑选状态为 READY 且在出生地点的无人机
                    drone_physical_status = next(
                        (drone for drone in self.drone_physical_status if drone.drone_work_state == DronePhysicalStatus.READY and self.des_pos_reached(birth_pos, drone.pos.position, 0.5) and drone.remaining_capacity >= 30), None)
                    # 如果没有找到符合条件的无人机，直接返回 None
                    # 添加条件，不得悬挂
                    if drone_physical_status is None:
                        print(f"订单{waybill['index']}, {car_sn}没有找到合适的无人机，只能进行空车移动")
                        is_empty_car = True  # 设置为空车行走
                        state = WorkState.MOVE_CAR_TO_LEAVING_POINT
                    else:
                        drone_sn = drone_physical_status.sn
                        print(f"订单{waybill['index']}, {car_sn}找到无人机{drone_sn}")
                        state = WorkState.MOVE_DRONE_ON_CAR
                else:
                    print(f"订单{waybill['index']}, {car_sn}当前小车有无人机")
                    drone_physical_status = next(
                        (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                    if drone_physical_status.remaining_capacity < 30:
                        print(f"订单{waybill['index']}, {car_sn}当前小车无人机电量不足")
                        # 挑选无人机，其状态是ready且在出生地点,电量充足
                        drone_physical_status = next(
                            (drone for drone in self.drone_physical_status if drone.drone_work_state == DronePhysicalStatus.READY and self.des_pos_reached(birth_pos, drone.pos.position, 0.5) and drone.remaining_capacity >= 30), None)
                        if drone_physical_status is None:
                            print(f"订单{waybill['index']}, {car_sn}其他合适的无人机也没电了，进入换电池")
                            state = WorkState.DRONE_BATTERY_REPLACEMENT
                        else:
                            # 回收飞机预计3s，挪合适飞机预计3s
                            self.drone_retrieve(
                                drone_sn, car_sn, 3, WorkState.MOVE_DRONE_ON_CAR)
                            drone_sn = drone_physical_status.sn
                            print(f"订单{waybill['index']}, {car_sn}换合适的无人机{drone_sn}")
                            state = WorkState.MOVE_DRONE_ON_CAR
                    elif drone_physical_status.bind_cargo_id:  # 额外检查，防止挂两个货物
                        print(f"无人机{drone_sn}已绑定货物，可能会导致出错")
                        state = WorkState.MOVE_CARGO_IN_DRONE
                    else:
                        print(f"订单{waybill['index']}, car_sn:{car_sn}车上有电量充足的无人机，进入绑货物")
                        state = WorkState.MOVE_CARGO_IN_DRONE
                print(f"car_sn:{car_sn},drone_sn:{drone_sn},waybill:{waybill['cargoParam']['index']}")
                # print(f"loading_pos:{loading_pos},\n takeoff_pos:{takeoff_pos}\n, landing_pos:{landing_pos}\n,flying_height:{flying_height}")
            elif state == WorkState.MOVE_DRONE_ON_CAR:
                MOVE_DRONE_ON_CAR_start = rospy.Time.now()
                # 将无人机移动到车上
                car_physical_status = next(
                    (car for car in self.car_physical_status if car.sn == car_sn), None)
                car_pos = car_physical_status.pos.position
                drone_physical_status = next(
                            (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                if (self.des_pos_reached(loading_pos, car_pos, 1) and car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY) and drone_physical_status.drone_work_state == DronePhysicalStatus.READY and self.des_pos_reached(birth_pos, drone_physical_status.pos.position, 0.5):
                    print(f"订单{waybill['index']},car_sn:{car_sn},drone_sn:{drone_sn}:开始挪机")
                    self.move_drone_on_car(
                        car_sn, drone_sn, 3.0, WorkState.MOVE_CARGO_IN_DRONE)
                    MOVE_DRONE_ON_CAR_time = (rospy.Time.now() - MOVE_DRONE_ON_CAR_start).to_sec()
                    print(f"挪机用时:{MOVE_DRONE_ON_CAR_time}秒，开始绑货物")
                    state =  WorkState.MOVE_CARGO_IN_DRONE
            elif state == WorkState.DRONE_BATTERY_REPLACEMENT:
                # 检查一下车的位置
                DRONE_BATTERY_REPLACEMENT_start = rospy.Time.now()
                car_physical_status = next(
                    (car for car in self.car_physical_status if car.sn == car_sn), None)
                car_pos = car_physical_status.pos.position
                # 换电池
                if(self.des_pos_reached(loading_pos, car_pos, 1) and car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                    print(f"订单{waybill['index']},car_sn:{car_sn},drone_sn:{drone_sn}:换电池")
                    self.battery_replacement(
                        drone_sn, 10,  WorkState.MOVE_CAR_TO_LEAVING_POINT)
                    drone_physical_status = next(
                        (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                    print(f"换电后无人机{drone_sn}电量为:{drone_physical_status.remaining_capacity}")
                    # print(f"car_sn:{car_sn},drone_sn:{drone_sn}:回收无人机")
                    # self.drone_retrieve(
                    #     drone_sn, car_sn, 3,  WorkState.MOVE_CAR_TO_LEAVING_POINT)
                    # drone_sn = ''
                    DRONE_BATTERY_REPLACEMENT_time = (rospy.Time.now() - DRONE_BATTERY_REPLACEMENT_start).to_sec()
                    print(f"无人机换电用时:{DRONE_BATTERY_REPLACEMENT_time}秒，开始进入绑定外卖环节")
                    state = WorkState.MOVE_CARGO_IN_DRONE
            elif state == WorkState.MOVE_CARGO_IN_DRONE:
                MOVE_CARGO_IN_DRONE_start = rospy.Time.now()
                if self.waybill_count_start == 1:
                    print("第一单绑外卖前需要休眠3s")
                    rospy.sleep(3)
                print(f"订单{waybill['index']},car_sn:{car_sn},drone_sn:{drone_sn}:开始绑外卖")
                cargo_bind_time_ms = int(rospy.get_time() * 1000) - self.running_start_time_ms
                print(f"订单{waybill['index']}: 货物绑定时间 : {cargo_bind_time_ms} - 毫秒戳")
                print(f"订单{waybill['index']}: 订单时间 orderTime: {waybill['orderTime']} - 毫秒戳")
                print(f"订单{waybill['index']}: 超时时间 timeout: {waybill['timeout']} - 毫秒戳")
                car_physical_status = next(
                        (car for car in self.car_physical_status  if self.des_pos_reached(car.pos.position, loading_pos, 0.5) and car.car_work_state == CarPhysicalStatus.CAR_READY), None)
                drone_sn = car_physical_status.drone_sn
                # 挂外卖
                # 从订单信息waybill中提取对应的外卖ID
                cargo_id = waybill['cargoParam']['index']

                self.move_cargo_in_drone(cargo_id, drone_sn, 10.0)
                drone_physical_status = next(
                    (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                bind_cargo_id = drone_physical_status.bind_cargo_id

                if bind_cargo_id == 0:
                    # 以防万一，一般不会出现这种情况
                    print(f"订单{waybill['index']},bind_cargoID = 0, 未到orderTime/超过timeout, 回收无人机，开始进入移车环节")
                    # 回收飞机预计3s，挪合适飞机预计3s
                    self.drone_retrieve(
                        drone_sn, car_sn, 3, WorkState.MOVE_DRONE_ON_CAR)
                    is_empty_car = True  # 设置为空车行走
                    state = WorkState.MOVE_CAR_TO_LEAVING_POINT
                else:
                    MOVE_CARGO_IN_DRONE_time = (rospy.Time.now() - MOVE_CARGO_IN_DRONE_start).to_sec()
                    print(f"car_sn:{car_sn},drone_sn:{drone_sn}:cargo_id:{cargo_id}; bind_cargo_id:{bind_cargo_id}; 绑外卖用时:{MOVE_CARGO_IN_DRONE_time}秒，开始进入移车环节")
                    state = WorkState.MOVE_CAR_TO_LEAVING_POINT
            elif state == WorkState.MOVE_CAR_TO_LEAVING_POINT:
                # 等待前一单无人机起飞完成
                if self.waybill_count_start > 1:
                    print(f"订单{waybill['index']}, car_sn:{car_sn}:等待前一单无人机起飞...")
                    self.drone_takeoff_semaphore.acquire()  # 阻塞，直到无人机成功起飞(-1)
                print(f"订单{waybill['index']}, car_sn:{car_sn}:等待前一单无人机降落...")
                self.drone_landing_semaphore.acquire()  # 阻塞，直到降落信号量被释放(-1)

                # 移车估计用时15s
                MOVE_CAR_TO_LEAVING_POINT_time = (rospy.Time.now() - dispatching_start_time).to_sec()
                print(f"car_sn:{car_sn}:前一单无人机已起飞，前前单无人机已降落，从订单开始到移车开始:{MOVE_CAR_TO_LEAVING_POINT_time}秒(观察指标),可能需要等待(准备周期)")
                if MOVE_CAR_TO_LEAVING_POINT_time < Preparation_Cycle:
                    rospy.sleep(Preparation_Cycle-MOVE_CAR_TO_LEAVING_POINT_time)
                else:
                    print(f"订单{waybill['index']}, car_sn:{car_sn}:移车前准备超时，可能需要调整时间")

                print(f"订单{waybill['index']}, car_sn:{car_sn}开始运动")
                MOVE_CAR_TO_LEAVING_POINT_start = rospy.Time.now()
                # 小车搭载挂外卖的无人机到达起飞点
                self.move_car_to_target_pos(car_list)
                car_physical_status = next(
                        (car for car in self.car_physical_status if car.sn == car_sn), None)
                drone_physical_status = next(
                        (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                
                while self.des_pos_reached(car_physical_status.pos.position, loading_pos, 0.5):
                    # print(f"car_sn:{car_sn}小车正在移动中...当前坐标:{car_physical_status.pos.position}")
                    rospy.sleep(0.5)  # 每隔时间检查一次位置
                    car_physical_status = next(
                        (car for car in self.car_physical_status if car.sn == car_sn), None)
                    drone_physical_status = next(
                        (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)

                while not self.des_pos_reached(car_physical_status.pos.position, takeoff_pos, 0.5):
                    # print(f"car_sn:{car_sn}小车正在移动中...当前坐标:{car_physical_status.pos.position}")
                    rospy.sleep(0.5)  # 每隔时间检查一次位置
                    car_physical_status = next(
                        (car for car in self.car_physical_status if car.sn == car_sn), None)
                    drone_physical_status = next(
                        (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                print(f"订单{waybill['index']},载无人机起飞的小车离开装货点, 开启下一单")
                self.order_semaphore.release()  # 释放信号量，允许下一单开始

                    
                MOVE_CAR_TO_LEAVING_POINT_time = (rospy.Time.now() - MOVE_CAR_TO_LEAVING_POINT_start).to_sec()
                print(f"订单{waybill['index']},载无人机起飞的小车到达起飞点, 该小车移动时间(开始运动到运动结束)为{MOVE_CAR_TO_LEAVING_POINT_time}")

                start_to_move_finish_time = (rospy.Time.now() - dispatching_start_time).to_sec()
                print(f"订单{waybill['index']},car_sn:{car_sn},drone_sn:{drone_sn}:从订单开始到移车结束: {start_to_move_finish_time}秒(观察指标)")
                if start_to_move_finish_time < Moving_car_cycle:
                    rospy.sleep(Moving_car_cycle-start_to_move_finish_time)
                    print(f"订单{waybill['index']},car_sn:{car_sn},drone_sn:{drone_sn}: 等待{Moving_car_cycle-start_to_move_finish_time}秒才释放下一单的开始/空单重复, 保证一个周期{Moving_car_cycle}s")
                else:
                    print(f"订单{waybill['index']},car_sn:{car_sn}: 准备时间和移车时间超时，大于一个周期，可能需要调整")

                if is_empty_car:
                    # 空车情况
                    self.loss_waybill += 1
                    is_empty_car = False
                    print(f"订单{waybill['index']},car_sn:{car_sn}空车行走移动完成，回到选择无人机的状态，本订单结束。开启下一单")
                    self.drone_takeoff_semaphore.release() # 释放起飞信号量(+1)
                    self.drone_landing_semaphore.release() # 释放降落信号量，以便下一个无人机可以继续降落(+1)
                    # 未到/超时情况，放弃处理，否则连锁反应
                    break
                else:
                    self.drone_landing_semaphore.release() # 释放降落信号量，以便下一个小车可以继续降落(+1)
                    state = WorkState.RELEASE_DRONE_OUT
            elif state == WorkState.RELEASE_DRONE_OUT:
                # 非空车情况：放飞无人机
                # 查询无人机当前的位置
                car_physical_status = next(
                    (car for car in self.car_physical_status if car.sn == car_sn), None)
                drone_physical_status = next(
                        (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                drone_pos = drone_physical_status.pos.position
                # 判断无人机是否到达起飞地点
                if (self.des_pos_reached(drone_pos, takeoff_pos, 1) and car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                    print(f"订单{waybill['index']}, car_sn:{car_sn},drone_sn:{drone_sn}: 进入放飞无人机阶段")
                    pre_time = (rospy.Time.now() - dispatching_start_time).to_sec()
                    print(f"订单{waybill['index']}, car_sn:{car_sn},drone_sn:{drone_sn}: 前期准备工作（到放飞无人机）花费的时间{pre_time}")
                    start_pos = (drone_pos.x, drone_pos.y, flying_height)
                    middle_pos = (
                        waybill['targetPosition']['x'], waybill['targetPosition']['y'], flying_height)
                    start_to_middle_route = navigate_with_astar(start_pos, middle_pos, step_size=5, threshold=5)
                    print(f"car_sn:{car_sn},drone_sn:{drone_sn}:路线{start_to_middle_route}")
                    # 将每个路径点转换为 Position 对象
                    position_list = [Position(x, y, z) for (x, y, z) in start_to_middle_route]
                    end_pos = Position(
                        waybill['targetPosition']['x'],
                        waybill['targetPosition']['y'],
                        waybill['targetPosition']['z']-5)
                    route = position_list + [end_pos]
                    # 计算总距离
                    total_distance = 0
                    for i in range(1, len(route)):
                        total_distance += self.calculate_distance(route[i-1], route[i])
                    # 无人机按照路径飞行
                    takeoff_time = rospy.Time.now()
                    self.fly_one_route(
                        drone_sn, route, 10.0, 60, WorkState.RELEASE_CARGO, is_departure=True)
                    state = WorkState.RELEASE_CARGO
            elif state == WorkState.RELEASE_CARGO:
                des_pos = Position(
                    waybill['targetPosition']['x'],
                    waybill['targetPosition']['y'],
                    waybill['targetPosition']['z'])
                drone_physical_status = next(
                        (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                drone_pos = drone_physical_status.pos.position
                # 释放货物
                if (self.des_pos_reached(des_pos, drone_pos, 2) and drone_physical_status.drone_work_state ==  DronePhysicalStatus.READY):
                    # 记录起飞后到送货物时间
                    cargo_time = (rospy.Time.now() - takeoff_time).to_sec()
                    speed = total_distance/cargo_time
                    self.release_cargo(
                        drone_sn, 5.0, WorkState.RELEASE_DRONE_RETURN)

                    bill_state = "成功"
                    self.waybill_count_finish += 1
                    # print("********************")
                    # print("以下打印外卖送达后信息")
                    print(f"订单{waybill['index']}, 外卖送达 - car_sn:{car_sn},drone_sn:{drone_sn}:外卖送{bill_state}啦！！！！！cargo-time用时:{cargo_time}")
                    delivery_time_ms = int(rospy.get_time() * 1000) - self.running_start_time_ms
                    print(f"订单{waybill['index']}, 货物送达时间戳: {delivery_time_ms} 毫秒时间戳")
                    waiting_time_1 = round(5 * (Moving_car_cycle + 1) - cargo_time, 1)
                    rospy.sleep(waiting_time_1)
                    waiting_time_2 = waiting_time_1
                    rospy.sleep(waiting_time_2)
                    # print(f"car_sn:{car_sn},drone_sn:{drone_sn}:飞机送货耗时: {cargo_time} 秒")
                    # print(f"car_sn:{car_sn},drone_sn:{drone_sn}:总距离: {total_distance:.2f},速度: {speed}")
                    # print(f"car_sn:{car_sn},drone_sn:{drone_sn}:送货等待{waiting_time_1}秒")
                    # print(f"car_sn:{car_sn},drone_sn:{drone_sn}:返航等待{waiting_time_2}秒")
                    state =  WorkState.RELEASE_DRONE_RETURN
            elif state == WorkState.RELEASE_DRONE_RETURN:
                # 无人机返回机场
                drone_physical_status = next(
                        (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                # 检查无人机是否存在
                if drone_physical_status is None:
                    print(f"订单{waybill['index']}, 无法找到编号为 {drone_sn} 的无人机，任务无法继续")
                    return  # 或者可以采取其他的错误处理逻辑
                drone_pos = drone_physical_status.pos.position
                if (self.des_pos_reached(des_pos, drone_pos, 2) and drone_physical_status.drone_work_state == DronePhysicalStatus.READY):
                    print(f"订单{waybill['index']}, car_sn:{car_sn},drone_sn:{drone_sn}:无人机返回机场")
                    route = route[1:-1][::-1]  # 使用切片翻转列表
                    route = [Position(point.x, point.y, flying_height-18) for point in route]
                    end_pos_1 = Position(landing_pos.x, landing_pos.y, flying_height-18)
                    end_pos_2 = Position(landing_pos.x, landing_pos.y, landing_pos.z-4)
                    route = route + [end_pos_1,end_pos_2]
                    # 飞到降落点上空，等待降落
                    rospy.sleep(0.1)
                    back_start_time = rospy.Time.now()
                    self.fly_one_route(
                        drone_sn, route, 10, 60, WorkState.DRONE_LANDING, is_departure=False) 
                    state =  WorkState.DRONE_LANDING
            elif state == WorkState.DRONE_LANDING:
                drone_physical_status = next(
                    (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                drone_pos = drone_physical_status.pos.position
                if self.des_pos_reached(end_pos_2, drone_pos, 0.5):
                    back_time = (rospy.Time.now() - back_start_time).to_sec()
                    if not self.is_landing_blocked:
                        self.drone_landing_semaphore.acquire()  # 阻止小车移动
                        self.is_landing_blocked = True  # 标记信号量已经被获取
                        print(f"订单{waybill['index']}, car_sn:{car_sn},drone_sn:{drone_sn}:降落信号量已获取，当前信号量值已减少，防止小车移动")
                    if flag:
                        print(f"订单{waybill['index']}, car_sn:{car_sn},drone_sn:{drone_sn}:飞机返回耗时: {back_time}秒")
                        flag = False
                if self.des_pos_reached(landing_pos, drone_pos, 1) and drone_physical_status.drone_work_state == DronePhysicalStatus.READY:
                    # 处理信号量
                    print(f"订单{waybill['index']}, car_sn:{car_sn},drone_sn:{drone_sn}:已成功降落，释放降落信号量，允许小车移动")
                    self.drone_landing_semaphore.release()  # 释放信号量，允许小车移动(+1)
                    self.is_landing_blocked = False  # 重置标志位
                    back_land_time = (rospy.Time.now() - back_start_time).to_sec()
                    print("********************")
                    print("以下打印无人机降落后信息")
                    print(f"订单{waybill['index']}, car_sn:{car_sn},drone_sn:{drone_sn}:waybill_cargo_id:{cargo_id},loading_pos:{loading_pos}, takeoff_pos:{takeoff_pos}, landing_pos:{landing_pos},flying_height:{flying_height}")
                    print(f"外卖送{bill_state}啦！！！！！")
                    print(f"前期准备工作花费的时间{pre_time}")
                    print(f"飞机送货耗时: {cargo_time} 秒")
                    print(f"总距离: {total_distance:.2f}")
                    print(f"送货等待{waiting_time_1}秒")
                    print(f"返航等待{waiting_time_2}秒")
                    print(f"飞机返回耗时: {back_time}秒")
                    print(f"飞机返回着陆耗时: {back_land_time}秒")
                    print(f"飞机着陆耗时(pos2->landing_pos): {back_land_time-back_time}秒")
                    print(f"来回的差值{back_land_time-cargo_time}")
                    print(f"已开始的订单数{self.waybill_count_start}, 丢弃订单数{self.giveup_waybill}, 失败订单数{self.loss_waybill}, 编号Waybill ID: {waybill['index']}")
                    print(f"订单时间 orderTime: {waybill['orderTime']} - 毫秒戳")
                    print(f"最佳送达时间 betterTime: {waybill['betterTime']} - 毫秒戳")
                    print(f"超时时间 timeout: {waybill['timeout']} - 毫秒戳")
                    print(f"货物绑定时间戳: {cargo_bind_time_ms} - 毫秒戳")
                    print(f"货物送达时间戳: {delivery_time_ms} - 毫秒戳")
                    print(f"已开始的总订单量{self.waybill_count_start}")
                    print(f"已完成的总订单量{self.waybill_count_finish}，当前的分数{self.score}")
                    print("当前时间(秒):", rospy.get_time() - self.running_start_time)
                    print("无人机降落完成，允许小车继续移动。")
                    print("********************")
                    # print(f"看看当前事件是啥{self.events}")
                    break
                        
    # 状态流转主函数
    def running(self):
        print("开始运行")
        while self.car_physical_status is None:
            print("等待小车状态初始化...")
            rospy.sleep(1.0)  # 等待 1 秒钟再检查
        self.running_start_time = rospy.get_time()  # 使用 rospy 获取当前时间
        print(f"running start_time:{self.running_start_time}")
        # 循环运行，直到达到 3600 秒
        while not rospy.is_shutdown():
            # 获取当前经过的时间
            elapsed_time = rospy.get_time() - self.running_start_time
            # 判断是否超过 3600 秒
            if elapsed_time >= 3600:
                rospy.loginfo("Time is up! 3600 seconds have passed.")
                break
            # 判断是否所有的小车都已经处于 CAR_READY 状态
            all_cars_ready = all(
                cps.car_work_state == CarPhysicalStatus.CAR_READY for cps in self.car_physical_status
            )
            # 根据判断结果执行不同的逻辑
            if all_cars_ready:
                print("所有小车都已准备好。")
                break
            else:
                rospy.sleep(3)
                print("还有小车未准备好。")

        print(self.state)
        self.sys_init()
        print(self.state)
        self.test_map_query()
        self.waybill_count_start = 0
        self.waybill_count_finish = 0
        # 无人机出生点
        birth_pos = Position(185,425,-16)
        # 装载点
        loading_pos = Position(
            self.loading_cargo_point['x'],
            self.loading_cargo_point['y'],
            self.loading_cargo_point['z']
        )
        # 起飞点
        takeoff_pos = Position(187,431,-16)
        # 降落点
        landing_pos = Position(181,434,-16)
        # 定义循环路径点
        points = [
            Position(187,431,-16),
            Position(181,434,-16),
            Position(190,440,-16),
            Position(199,434,-16),
            Position(195,425,-16),
            loading_pos]
        # 小车位置信息
        car_info = [
            ("SIM-MAGV-0001", 5, points),
            ("SIM-MAGV-0002", 0, points),
            ("SIM-MAGV-0003", 2, points),
            ("SIM-MAGV-0004", 4, points),
            ("SIM-MAGV-0005", 1, points),
            ("SIM-MAGV-0006", 3, points)
        ]
        # 初始化车辆对象
        car_list = [
            Car(car_sn, index, points)
            for car_sn, index, points in car_info
        ]
        print("开始初始化")
        threads = []
        for car in car_list:
            print(f"开始初始化{car.car_sn}")
            thread = threading.Thread(
                target=self.init_car, args=(car,)
            )
            threads.append(thread)
        for thread in threads:  
            thread.start()
        # 等待所有线程完成
        for thread in threads:
            thread.join()
        rospy.sleep(15)
        print("用时15s初始化完成")
        # 确保在循环开始前子列表已经按照betterTime+timeout排序
        groups = self.waybill_classification()
        iterators = [iter(group) for group in groups]
        # # 循环提取每个子列表的一个元素，直到所有子列表都为空
        flying_height_list = [-86, -92, -98]
        flying_height_iterator = itertools.cycle(flying_height_list)
        while not rospy.is_shutdown() and iterators:
            flying_height = next(flying_height_iterator)
            # flying_height = -64
            # 使用副本循环，因为可能会移除空的子列表
            # 创建每个子订单组的进程
            threads = []
            # 每个迭代器对应一个已经排序的子列表
            for it in iterators[:]:
                while True:  # 在每个迭代器中使用 while 循环
                    try:
                        # 尝试从当前迭代器中提取一个订单
                        print("********************")
                        waybill = next(it)
                        print("当前时间(秒):", rospy.get_time() - self.running_start_time)
                        print(f"提取订单-waybill如下:{waybill['index']}")
                        # 初始化ros变量
                        state = WorkState.SELACT_WAYBILL_CAR_DRONE
                        # 在 running() 方法中为每个线程初始化 is_empty_car
                        is_empty_car = False     # 初始化为 False，表示默认不是空车
                        bind_cargo_attempts = 0  # 用于跟踪绑定货物的尝试次数

                        select_start_time_ms = int(rospy.get_time() * 1000) - self.running_start_time_ms
                        if self.waybill_count_start > 1 and (select_start_time_ms > (waybill['orderTime'] + 120000)) and ((select_start_time_ms + 15000 > (waybill['timeout'])) or (select_start_time_ms + 127000 > ((waybill['timeout'] - waybill['betterTime'])/8)+waybill['betterTime'])):
                            # 丢弃这一单，直接开始下一单
                            # 不同组的单间隔orderTime为120-150秒左右
                            self.giveup_waybill += 1
                            print(f"当前订单{waybill['index']}不符合绑定要求，直接放弃该订单，开始提取下一单")
                            print(f"当前订单提取时间: {select_start_time_ms}")
                            print(f"订单时间 orderTime: {waybill['orderTime']} - 毫秒戳")
                            print(f"最佳送达时间 betterTime: {waybill['betterTime']} - 毫秒戳")
                            print(f"超时时间 timeout: {waybill['timeout']} - 毫秒戳")
                            # rospy.sleep(1)
                            continue
                        else:
                            print(f"当前订单{waybill['index']}符合绑定要求，开启处理线程")
                            print(f"看看当前事件是啥{self.events}")
                            print("********************")
                            thread = threading.Thread(
                                target=self.dispatching, 
                                args=(car_list, loading_pos, birth_pos, takeoff_pos, landing_pos, waybill, flying_height, state, is_empty_car, bind_cargo_attempts)
                            )
                            threads.append(thread)
                            thread.start()
                            rospy.sleep(Moving_car_cycle+1)     # 每多少秒周期提取并处理一单订单
                        break  # 成功处理完一个订单后，退出内部循环
                    except StopIteration:
                        # 如果迭代器已经耗尽，从列表中移除
                        iterators.remove(it)
                        break
            # rospy.sleep(35.3)
            # # 等待所有线程完成
            # for thread in threads:
            #     thread.join()

            
        rospy.sleep(1.0)
        print(
            'Total waybill finished: ',
            self.waybill_count_finish,
            ', Total score: ',
            self.score)


if __name__ == '__main__':
    race_demo = DemoPipeline()
    race_demo.running()