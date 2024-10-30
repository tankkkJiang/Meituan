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

def get_millis():
    """
    获取当前时间的毫米戳。
    """
    return int(rospy.get_time() * 1000)

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
            queue_size=10)
        self.map_client = rospy.ServiceProxy('query_voxel', QueryVoxel)
        # 读取配置文件和信息
        with open('/config/config.json', 'r') as file:
            self.config = json.load(file)
        self.drone_infos = self.config['taskParam']['droneParamList']
        self.car_infos = self.config['taskParam']['magvParamList']
        self.loading_cargo_point = self.config['taskParam']['loadingCargoPoint']
        self.map_boundary = self.config['taskParam']['mapBoundaryInfo']

        self.waybill_infos = self.config['taskParam']['waybillParamList']
        # 在派发前按 betterTime + timeout 排序waybills
        self.waybill_infos.sort(key=lambda x: x['betterTime'] + x['timeout'])

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
        self.move_cargo_in_drone_millis = None  # 初始化挂餐时间，最后可以打印
        self.delivery_time_millis = None        # 初始化送达时间，最后可以打印
        self.waybill_start_time_millis = None
        self.order_semaphore = threading.Semaphore(0)  # 初始不可用

    # 仿真回调函数，获取实时信息
    def panoramic_info_callback(self, panoramic_info):
        self.car_physical_status = panoramic_info.cars
        self.drone_physical_status = panoramic_info.drones
        self.bills_status = panoramic_info.bills
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

    # 移动地面车辆的函数
    def move_car_with_start_and_end(self, car_sn, start, end, time_est, next_state):
        # print("开始移动")
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = car_sn
        msg.car_route_info.way_point.append(start)
        msg.car_route_info.way_point.append(end)
        msg.car_route_info.yaw = 0.0
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        
    # 检测位置到达的函数
    def des_pos_reached(self, des_pos, cur_pos, threshold):
        des = np.array([des_pos.x, des_pos.y, des_pos.z])
        cur = np.array([cur_pos.x, cur_pos.y, cur_pos.z])
        return np.linalg.norm(np.array(des - cur)) < threshold

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
    def fly_one_route(self, drone_sn, route, speed, time_est, next_state):
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
        rospy.sleep(time_est)
        state = next_state

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
            middle = Position(183, 438, -16)
            msg.car_route_info.way_point.append(start)
            msg.car_route_info.way_point.append(middle)
            msg.car_route_info.way_point.append(end)

        elif car_sn == "SIM-MAGV-0005":
            print("移动5号车")
            middle = Position(190, 438, -16)
            msg.car_route_info.way_point.append(start)
            msg.car_route_info.way_point.append(middle)
            msg.car_route_info.way_point.append(end)

        # 中间的两边也要绕一下
        elif car_sn == "SIM-MAGV-0003":
            print("移动3号车")
            middle_1 = Position(189, 446, -16)
            middle_2 = Position(189, 438, -16)
            msg.car_route_info.way_point.append(start)
            msg.car_route_info.way_point.append(middle_1)
            msg.car_route_info.way_point.append(middle_2)
            msg.car_route_info.way_point.append(end)

        elif car_sn == "SIM-MAGV-0006":
            print("移动6号车")
            middle_1 = Position(193, 446, -16)
            middle_2 = Position(193, 438, -16)
            msg.car_route_info.way_point.append(start)
            msg.car_route_info.way_point.append(middle_1)
            msg.car_route_info.way_point.append(middle_2)
            msg.car_route_info.way_point.append(end)
        
        elif car_sn == "SIM-MAGV-0004":
            print("移动4号车")
            msg.car_route_info.way_point.append(start)
            msg.car_route_info.way_point.append(end)

        msg.car_route_info.yaw = 0.0
        self.cmd_pub.publish(msg)

    # 移动单辆小车
    def move_car(self, car):
        car_physical_status = next(
            (cps for cps in self.car_physical_status if cps.sn == car.car_sn), None)
        car_pos = car_physical_status.pos.position
        self.move_car_with_start_and_end(
            car.car_sn, car_pos, car.target_pos, 0, WorkState.SELACT_WAYBILL_CAR_DRONE
        )
        # 更新小车当前位置和目标位置
        car.set_target()
    
    # 小车按照循环点移动
    def move_car_to_target_pos(self, car_list):
        print("正在调用循环小车移动")
        threads = []
        # 创建所有线程
        for car in car_list:
            thread = threading.Thread(
                target=self.move_car, args=(car,)
            )
            threads.append(thread)
        for thread in threads:  
            thread.start()
        # 等待所有线程完成
        for thread in threads:
            thread.join()
        # print("小车位置初始化完成")

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
        # 在每个组内按 'betterTime' + 'timeout' 进行排序
        for group in groups:
            group.sort(key=lambda x: x['betterTime'] + x['timeout'])
        # 现在根据每个组中第一个条目的 'betterTime' + 'timeout' 对所有组进行排序，如果组不为空
        sorted_groups = sorted(groups, key=lambda g: g[0]['betterTime'] + g[0]['timeout'] if g else float('inf'))
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
    def dispatching(self, car_list, loading_pos, birth_pos, takeoff_pos, landing_pos, waybill, flying_height, state):       
        flag = True
        self.waybill_start_time_millis = get_millis()
        self.waybill_count += 1
        print(f"订单数{self.waybill_count}: Begin to dispatch")
        while not rospy.is_shutdown():
            if state == WorkState.SELACT_WAYBILL_CAR_DRONE:
                if self.waybill_count > 1:  # 假设waybill_count用于跟踪订单数
                    print("正在等待第一单完成...")
                    self.order_semaphore.acquire()  # 阻塞，直到第一单释放信号量
                print(f"订单数{self.waybill_count}：小车无人机初始化")
                dispatching_start_time = rospy.Time.now() 
                car_physical_status = next(
                    (car for car in self.car_physical_status if self.des_pos_reached(car.pos.position, loading_pos, 1) and car.car_work_state == CarPhysicalStatus.CAR_READY), None)
                car_sn = car_physical_status.sn 
                drone_sn = car_physical_status.drone_sn
                # 挑选无人机
                if drone_sn == '':
                    print(f"{car_sn}挑选无人机，当前小车没有无人机")
                    # 遍历无人机列表，挑选状态为 READY 且在出生地点的无人机
                    drone_physical_status = next(
                        (drone for drone in self.drone_physical_status if drone.drone_work_state == DronePhysicalStatus.READY and self.des_pos_reached(birth_pos, drone.pos.position, 0.5) and drone.remaining_capacity >= 30), None)
                    # 如果没有找到符合条件的无人机，直接返回 None
                    # 添加条件，不得悬挂
                    if drone_physical_status is None:
                        print(f"{car_sn}没有找到合适的无人机")
                        rospy.sleep(35)
                        state = WorkState.MOVE_CAR_TO_LEAVING_POINT
                    else:
                        drone_sn = drone_physical_status.sn
                        print(f"{car_sn}找到无人机{drone_sn}")
                        rospy.sleep(15)
                        state = WorkState.MOVE_DRONE_ON_CAR
                else:
                    print(f"{car_sn}当前小车有无人机")
                    drone_physical_status = next(
                        (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                    if drone_physical_status.remaining_capacity < 30:
                        print("电量不足")
                        # 挑选无人机，其状态是ready且在出生地点,电量充足
                        drone_physical_status = next(
                            (drone for drone in self.drone_physical_status if drone.drone_work_state == DronePhysicalStatus.READY and self.des_pos_reached(birth_pos, drone.pos.position, 0.5) and drone.remaining_capacity >= 30), None)
                        if drone_physical_status is None:
                            print("其他合适的无人机也没电了")
                            rospy.sleep(15)
                            state = WorkState.DRONE_BATTERY_REPLACEMENT
                        else:
                            print("换无人机")
                            self.drone_retrieve(
                                drone_sn, car_sn, 15, WorkState.MOVE_DRONE_ON_CAR)
                            drone_sn = drone_physical_status.sn
                            state = WorkState.MOVE_DRONE_ON_CAR
                    elif drone_physical_status.bind_cargo_id:  # 额外检查，防止挂两个货物
                        print(f"无人机{drone_sn}已绑定货物，可能会导致出错")
                        state = WorkState.MOVE_CARGO_IN_DRONE
                    else:
                        print("车上有电量充足的无人机")
                        rospy.sleep(20)
                        state = WorkState.MOVE_CARGO_IN_DRONE
                print(f"car_sn:{car_sn},drone_sn:{drone_sn},waybill:{waybill['cargoParam']['index']}")
                print(f"loading_pos:{loading_pos},\n takeoff_pos:{takeoff_pos}\n, landing_pos:{landing_pos}\n,flying_height:{flying_height}")
            elif state == WorkState.MOVE_DRONE_ON_CAR:
                # 将无人机移动到车上
                car_physical_status = next(
                    (car for car in self.car_physical_status if car.sn == car_sn), None)
                car_pos = car_physical_status.pos.position
                drone_physical_status = next(
                            (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                if (self.des_pos_reached(loading_pos, car_pos, 1) and car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY) and drone_physical_status.drone_work_state == DronePhysicalStatus.READY and self.des_pos_reached(birth_pos, drone_physical_status.pos.position, 0.5):
                    print(f"car_sn:{car_sn},drone_sn:{drone_sn}:开始挪机")
                    self.move_drone_on_car(
                        car_sn, drone_sn, 5.0, WorkState.MOVE_CARGO_IN_DRONE)
                    state =  WorkState.MOVE_CARGO_IN_DRONE
            elif state == WorkState.DRONE_BATTERY_REPLACEMENT:
                # 检查一下车的位置
                car_physical_status = next(
                    (car for car in self.car_physical_status if car.sn == car_sn), None)
                car_pos = car_physical_status.pos.position
                # 换电池
                if(self.des_pos_reached(loading_pos, car_pos, 1) and car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                    print(f"car_sn:{car_sn},drone_sn:{drone_sn}:换电池")
                    self.battery_replacement(
                        drone_sn, 15.0,  WorkState.MOVE_CAR_TO_LEAVING_POINT)
                    print(f"car_sn:{car_sn},drone_sn:{drone_sn}:回收无人机")
                    self.drone_retrieve(
                        drone_sn, car_sn, 5,  WorkState.MOVE_CAR_TO_LEAVING_POINT)
                    drone_sn = ''
                    state = WorkState.MOVE_CAR_TO_LEAVING_POINT
            elif state == WorkState.MOVE_CARGO_IN_DRONE:
                print(f"car_sn:{car_sn},drone_sn:{drone_sn}:绑外卖")
                car_physical_status = next(
                        (car for car in self.car_physical_status  if self.des_pos_reached(car.pos.position, loading_pos, 0.5) and car.car_work_state == CarPhysicalStatus.CAR_READY), None)
                drone_sn = car_physical_status.drone_sn
                # 挂外卖
                # 从订单信息waybill中提取对应的外卖ID
                cargo_id = waybill['cargoParam']['index']
                self.move_cargo_in_drone(cargo_id, drone_sn, 15.0)
                # 记录挂餐时间
                self.move_cargo_in_drone_millis = get_millis() - self.waybill_start_time_millis
                drone_physical_status = drone_physical_status = next(
                    (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                bind_cargo_id = drone_physical_status.bind_cargo_id
                print(f"car_sn:{car_sn},drone_sn:{drone_sn}:外卖订单{bind_cargo_id}")
                state = WorkState.MOVE_CAR_TO_LEAVING_POINT
            elif state == WorkState.MOVE_CAR_TO_LEAVING_POINT:
                print(f"car_sn:{car_sn},drone_sn:{drone_sn}:移动小车")
                # 小车搭载挂外卖的无人机到达起飞点
                # car_start_time = rospy.Time.now()
                self.move_car_to_target_pos(car_list)

                # 检查小车是否处于运动状态
                while True:
                    car_physical_status = next(
                        (car for car in self.car_physical_status if car.sn == car_sn), None)
                    if car_physical_status is None:
                        print("未找到对应的小车状态信息")
                        rospy.sleep(1)  # 短暂等待后再次检查
                        continue
                    
                    if car_physical_status.car_work_state == CarPhysicalStatus.CAR_RUNNING:
                        print("小车已经进入running状态。")
                        car_physical_status = next(
                            (car for car in self.car_physical_status if car.sn == car_sn), None)
                        car_pos = car_physical_status.pos.position
                        if not self.des_pos_reached(loading_pos, car_pos, 1):
                            print("小车位置已经不在装载点，正在移动...")
                            break  # 小车已经开始运动，跳出循环
                        else:
                            # print("虽然running状态但还未移动")
                            rospy.sleep(3)
                    else:
                        print("小车未在运动状态，等待小车开始移动...")
                        if self.waybill_count ==1:
                            rospy.sleep(10)
                            self.move_car_to_target_pos(car_list)
                            print("如果第一次，则重复运行一次循环点移动")
                        rospy.sleep(1)  # 等待一秒再检查小车状态

                while not car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                    print("小车正在移动中...")
                    rospy.sleep(1)  # 每秒检查一次位置不符合，有可能
                    car_physical_status = next(
                        (car for car in self.car_physical_status if car.sn == car_sn), None)

                print("小车移动完毕")
                self.order_semaphore.release()  # 释放信号量，允许第二单开始
                # rospy.sleep(3)
                state = WorkState.RELEASE_DRONE_OUT
            elif state == WorkState.RELEASE_DRONE_OUT:
                # 放飞无人机
                # 查询无人机当前的位置
                car_physical_status = next(
                    (car for car in self.car_physical_status if car.sn == car_sn), None)
                drone_physical_status = next(
                        (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                drone_pos = drone_physical_status.pos.position
                # 判断无人机是否到达起飞地点
                if (self.des_pos_reached(drone_pos, takeoff_pos, 1) and car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                    print(f"car_sn:{car_sn},drone_sn:{drone_sn}:准备放飞无人机")
                    pre_time = (rospy.Time.now() - dispatching_start_time).to_sec()
                    print(f"car_sn:{car_sn},drone_sn:{drone_sn}:前期准备工作花费的时间{pre_time}")
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
                    cargo_start_time = rospy.Time.now()
                    self.fly_one_route(
                        drone_sn, route, 10.0, 60, WorkState.RELEASE_CARGO)
                    # 等待并检查无人机的状态
                    while drone_physical_status.drone_work_state != DronePhysicalStatus.FLYING:
                        rospy.sleep(1)  # 每次检查前等待1秒
                        # 获取更新的无人机状态
                        drone_physical_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                        print(f"car_sn:{car_sn},drone_sn:{drone_sn}, drone_physical_status.drone_work_state{drone_physical_status.drone_work_state}")
                        if drone_physical_status.drone_work_state == DronePhysicalStatus.FLYING:
                            print(f"car_sn:{car_sn},drone_sn:{drone_sn}: 无人机正在飞行。")
                            break
                        elif drone_physical_status.drone_work_state == DronePhysicalStatus.LANDING:
                            print(f"car_sn:{car_sn},drone_sn:{drone_sn}: 无人机已经降落。")
                            break
                        elif drone_physical_status.drone_work_state == DronePhysicalStatus.ERROR:
                            print(f"car_sn:{car_sn},drone_sn:{drone_sn}: 无人机已经撞毁。")
                            break
                        elif drone_physical_status.drone_work_state == DronePhysicalStatus.TAKEOFF:
                            print(f"car_sn:{car_sn},drone_sn:{drone_sn}: 无人机起飞中。")
                        print(f"car_sn:{car_sn},drone_sn:{drone_sn}: 等待无人机开始飞行。")
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
                    cargo_time = (rospy.Time.now() - cargo_start_time).to_sec()
                    speed = total_distance/cargo_time
                    self.release_cargo(
                        drone_sn, 5.0, WorkState.RELEASE_DRONE_RETURN)
                    # 记录送达时间
                    self.delivery_time_millis = get_millis() - self.waybill_start_time_millis
                    print("self.delivery_time_millis", self.delivery_time_millis)

                    bill_state = "成功"
                    # print("********************")
                    # print("以下打印外卖送达后信息")
                    print(f"外卖送达 - car_sn:{car_sn},drone_sn:{drone_sn}:外卖送{bill_state}啦！！！！！")
                    waiting_time_1 = round(162.75-cargo_time, 1)
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
                drone_pos = drone_physical_status.pos.position
                if (self.des_pos_reached(des_pos, drone_pos, 2) and drone_physical_status.drone_work_state == DronePhysicalStatus.READY):
                    print(f"car_sn:{car_sn},drone_sn:{drone_sn}:无人机返回机场")
                    route = route[1:-1][::-1]  # 使用切片翻转列表
                    route = [Position(point.x, point.y, flying_height-18) for point in route]
                    end_pos_1 = Position(landing_pos.x, landing_pos.y, flying_height-18)
                    end_pos_2 = Position(landing_pos.x, landing_pos.y, landing_pos.z-5)
                    route = route + [end_pos_1,end_pos_2]
                    # 飞到降落点上空，等待降落
                    back_start_time = rospy.Time.now()
                    self.fly_one_route(
                        drone_sn, route, 10, 60, WorkState.DRONE_LANDING) 
                    state =  WorkState.DRONE_LANDING
            elif state == WorkState.DRONE_LANDING:
                drone_physical_status = next(
                    (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                drone_pos = drone_physical_status.pos.position
                if self.des_pos_reached(end_pos_2, drone_pos, 0.5):
                    back_time = (rospy.Time.now() - back_start_time).to_sec()
                    if flag:
                        print(f"car_sn:{car_sn},drone_sn:{drone_sn}:飞机返回耗时: {back_time}秒")
                        flag = False
                if self.des_pos_reached(landing_pos, drone_pos, 2) and drone_physical_status.drone_work_state == DronePhysicalStatus.READY:
                    back_land_time = (rospy.Time.now() - back_start_time).to_sec()
                    print("********************")
                    print("以下打印无人机降落后信息")
                    print(f"car_sn:{car_sn},drone_sn:{drone_sn}:waybill_cargo_id:{bind_cargo_id},loading_pos:{loading_pos}, takeoff_pos:{takeoff_pos}, landing_pos:{landing_pos},flying_height:{flying_height}")
                    print(f"外卖送{bill_state}啦！！！！！")
                    print(f"前期准备工作花费的时间{pre_time}")
                    print(f"飞机送货耗时: {cargo_time} 秒")
                    print(f"总距离: {total_distance:.2f}")
                    print(f"送货等待{waiting_time_1}秒")
                    print(f"返航等待{waiting_time_2}秒")
                    print(f"飞机返回耗时: {back_time}秒")
                    print(f"飞机返回着陆耗时: {back_land_time}秒")
                    print(f"飞机着陆耗时: {back_land_time-back_time}秒")
                    print(f"来回的差值{back_land_time-cargo_time}")
                    print(f"编号Waybill ID: {waybill['index']}")
                    print(f"订单时间 orderTime: {waybill['orderTime']} - 毫秒戳")
                    print(f"最佳送达时间 betterTime: {waybill['betterTime']} - 毫秒戳")
                    print(f"超时时间 timeout: {waybill['timeout']} - 毫秒戳")
                    print(f"挂餐时间：{self.move_cargo_in_drone_millis} - 毫米戳")
                    print(f"外卖送达时间: {self.delivery_time_millis} - 毫秒戳")
                    print(f"总订单量{self.waybill_count }，当前的分数{self.score}")
                    print("********************")
                    # print(f"看看当前事件是啥{self.events}")
                    break
                        
    # 状态流转主函数
    def running(self):
        print("开始运行")
        rospy.sleep(30.0)
        running_start_time = rospy.get_time()  # 使用 rospy 获取当前时间
        running_start_time_ms = running_start_time * 1000
        print(f"running start_time:{running_start_time}")
        # 循环运行，直到达到 3600 秒
        while not rospy.is_shutdown():
            # 获取当前经过的时间
            elapsed_time = rospy.get_time() - running_start_time
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
                rospy.sleep(5.0)
                print("还有小车未准备好。")

        print(self.state)
        self.sys_init()
        print(self.state)
        self.test_map_query()
        self.waybill_count = 0
        # 无人机出生点
        birth_pos = Position(185,425,-16)
        # 装载点
        loading_pos = Position(
            self.loading_cargo_point['x'],
            self.loading_cargo_point['y'],
            self.loading_cargo_point['z']
        )
        # 起飞点
        takeoff_pos = Position(183,431,-16)
        # 降落点
        landing_pos = Position(183,438,-16)
        # 定义循环路径点
        points = [
            Position(183,431,-16),
            Position(183,438,-16),
            Position(190,438,-16),
            Position(197,438,-16),
            Position(197,431,-16),
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
        rospy.sleep(30)
        print("初始化完成")

        # 确保在循环开始前子列表已经按照betterTime排序
        groups = self.waybill_classification()
        # # 打印排序后的结果
        # for index, group in enumerate(groups):
        #     print(f"分组 {index+1}:")  # 打印当前分组的序号
        #     for item in group:
        #         print(item)  # 打印分组内的每个元素

        # groups = self.group_waybills(self.waybill_infos, takeoff_pos)
        # 创建每个子列表的迭代器
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
                try:
                    # 尝试从当前迭代器中提取一个订单
                    print("********************")
                    print(f"看看当前事件是啥{self.events}")
                    waybill = next(it)
                    print("当前时间(秒):", rospy.get_time() - running_start_time)
                    print("提取订单: ")
                    print("waybill如下:", waybill)
                    print("********************")
                    # 初始化ros变量
                    state = WorkState.SELACT_WAYBILL_CAR_DRONE
                    thread = threading.Thread(
                        target=self.dispatching, 
                        args=(car_list, loading_pos, birth_pos, takeoff_pos, landing_pos, waybill, flying_height, state)
                    )
                    threads.append(thread)
                    thread.start()
                    rospy.sleep(65.1)     # 每多少秒周期提取并处理一单订单
                except StopIteration:
                    # 如果迭代器已经耗尽，从列表中移除
                    iterators.remove(it)
            # rospy.sleep(35.3)
            # # 等待所有线程完成
            # for thread in threads:
            #     thread.join()

            
        rospy.sleep(1.0)
        print(
            'Total waybill finished: ',
            self.waybill_count,
            ', Total score: ',
            self.score)


if __name__ == '__main__':
    print("tank333.py")
    race_demo = DemoPipeline()
    race_demo.running()
