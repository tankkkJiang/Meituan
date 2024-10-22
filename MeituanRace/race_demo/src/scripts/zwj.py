#!/usr/bin/env python3
import rospy
import json
import numpy as np
from enum import Enum
import threading
import time

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


class CarDirection(Enum):
    GO = 1
    BACK = 2


class Car:
    def __init__(self, car_sn, car_init_pos, drone_sn=None, route_id=0, state='empty'):
        self.state = state  # empty->go->back  bind->go_with_drone->back
        self.car_sn = car_sn
        self.car_init_pos = car_init_pos
        # 存储执行任务相关的信息
        self.task = {
            'drone_sn': drone_sn,
            'route_id': route_id,
        }


class Drone:
    def __init__(self, drone_sn, des_pos, time_est, next_state, car_sn=""):
        self.drone_sn = drone_sn
        self.des_pos = des_pos
        self.time_est = time_est
        self.next_state = next_state
        self.car_sn = car_sn


class DroneRoute:
    def __init__(self, height, max_drone_cnt, route_len=None, cur_drone_cnt=0, latest_drone_Sn=None,
                 latest_drone_sn_fly_time=0):
        self.height = height  # 航线爬升高度
        self.max_drone_cnt = max_drone_cnt  # 航线最大承载数量
        self.cur_drone_cnt = cur_drone_cnt  # 航线当前飞机数量
        # 待定属性：
        self.latest_drone_Sn = latest_drone_Sn  # 最近放飞的飞机sn（可以用来查该飞机的位置，计算已经飞行多少m？）
        self.latest_drone_sn_fly_time = latest_drone_sn_fly_time
        self.route_len = route_len


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
        self.map_boundary = self.config['taskParam']['mapBoundaryInfo']  # 地图边界
        self.waybill_infos = self.config['taskParam']['waybillParamList']  #
        # self.unloading_cargo_stations = self.config['taskParam']['unloadingCargoStationList'] # 这里先写死
        self.unloading_cargo_stations = [
            {'index': 1, 'name': '测试卸货点1', 'position': {'x': 146, 'y': 186, 'z': -34}},
            {'index': 2, 'name': '测试卸货点4', 'position': {'x': 508, 'y': 514, 'z': -22}},
            {'index': 3, 'name': '测试卸货点6', 'position': {'x': 490, 'y': 390, 'z': -22}}]
        self.drone_sn_list = [drone['droneSn'] for drone in self.drone_infos]
        self.car_sn_list = [car['magvSn'] for car in self.car_infos]
        self.peer_id = self.config['peerId']
        self.task_guid = self.config['taskParam']['guid']
        self.car_physical_status = None
        self.drone_physical_status = None
        self.bills_status = None
        self.score = None
        self.events = None
        # 订单按卸货点分类 送三个订单
        self.waybill_position = [[] for _ in range(3)]
        # 6条航线的状态信息
        self.route_state = [DroneRoute(64, 6), DroneRoute(74, 6), DroneRoute(114, 6)]
        self.car_list = []

    # 仿真回调函数，获取实时信息
    def panoramic_info_callback(self, panoramic_info):
        self.car_physical_status = panoramic_info.cars
        self.drone_physical_status = panoramic_info.drones
        self.bills_status = panoramic_info.bills
        self.score = panoramic_info.score
        self.events = panoramic_info.events

    # 系统初始化(按需)
    def sys_init(self):
        rospy.sleep(10.0)
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
            self.state = WorkState.MOVE_CAR_GO_TO_LOADING_POINT

    # 移动地面车辆的函数
    def move_car_with_start_and_end(
            self, car_sn, start, end, time_est, next_state):
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
        self.state = next_state

    # 检测位置到达的函数
    def des_pos_reached(self, des_pos, cur_pos, threshold):
        des = np.array([des_pos.x, des_pos.y, des_pos.z])
        cur = np.array([cur_pos.x, cur_pos.y, cur_pos.z])
        return np.linalg.norm(np.array(des - cur)) < threshold

    # 检测一个平面的距离
    def one_level_height_des_pos_reached(self, des_pos, cur_pos, threshold):
        des = np.array([des_pos.x, des_pos.y])
        cur = np.array([cur_pos.x, cur_pos.y])
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
        self.state = next_state

    # 网飞机上挂餐
    def move_cargo_in_drone(self, cargo_id, drone_sn, time_est):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_CARGO_IN_DRONE
        msg.binding_cargo.cargo_id = cargo_id
        msg.binding_cargo.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state = WorkState.MOVE_CAR_TO_LEAVING_POINT

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
        self.state = next_state

    # 抛餐函数
    def release_cargo(self, drone_sn, time_est, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_RELEASE_CARGO
        msg.drone_msg.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state = next_state

    # 换电函数
    def battery_replacement(self, drone_sn, time_est, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_BATTERY_REPLACEMENT
        msg.drone_msg.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state = next_state

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
        self.state = next_state

    '''这个move_car应该还是不能避免相撞
    需要把无人车路径拉远看一下情况 1 Position(185, 433, -16) 5  Position(195, 433, -16)
    '''

    def move_car(self, car_sn, start, end, car_dir):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = car_sn

        # 中间的直走
        if car_sn == "SIM-MAGV-0004":
            msg.car_route_info.way_point.append(start)
            msg.car_route_info.way_point.append(end)
        # 最外侧的两个绕路
        elif car_sn == "SIM-MAGV-0002":
            middle = Position(185.25, 425, -16)
            msg.car_route_info.way_point.append(start)
            msg.car_route_info.way_point.append(middle)
            msg.car_route_info.way_point.append(end)

        elif car_sn == "SIM-MAGV-0006":
            middle = Position(194.75, 425, -16)
            msg.car_route_info.way_point.append(start)
            msg.car_route_info.way_point.append(middle)
            msg.car_route_info.way_point.append(end)

        # 中间的两边也要绕一下
        elif car_sn == "SIM-MAGV-0001":
            middle = Position(187.5, 427, -16)
            msg.car_route_info.way_point.append(start)
            msg.car_route_info.way_point.append(middle)
            msg.car_route_info.way_point.append(end)


        elif car_sn == "SIM-MAGV-0005":
            middle = Position(192.5, 427, -16)
            msg.car_route_info.way_point.append(start)
            msg.car_route_info.way_point.append(middle)
            msg.car_route_info.way_point.append(end)

        msg.car_route_info.yaw = 0.0
        self.cmd_pub.publish(msg)

    def find_car_id(self, car_sn):
        car_id = [2, 0, 4, 1, 5, 3]
        return car_id[int(car_sn[-1]) - 1]

    def find_id_car(self, num):
        car_id = ["SIM-MAGV-0001", "SIM-MAGV-0002", "SIM-MAGV-0004"]
        return car_id[num]

    def find_id_drone(self, num):
        return 'SIM-DRONE-00' + str(num + 1).zfill(2)

    def waybill_classification(self):  # 这里的订单分类只分最优的4个卸货点
        for i in range(len(self.waybill_infos)):
            waybill = self.waybill_infos[i]
            key = str(int(waybill['targetPosition']['x'])) + ',' + str(int(waybill['targetPosition']['y'])) + ',' + str(int(waybill['targetPosition']['z']))
            willbill_dict = {'146,186,-34': 0, '508,514,-22': 1, '490,390,-22': 2, '430,184,-10': -1, '528,172,-20': -1,
                             '564,394,-16': -1}
            self.waybill_position[willbill_dict[key]].append(waybill)

    def get_next_drone(self):
        if self.last_drone_id == 29:
            return 0
        else:
            return self.last_drone_id + 1

    def init_car_arr(self):  # 最优小车顺序为4 1 5 2 6
        car_id = ["SIM-MAGV-0004", "SIM-MAGV-0001", "SIM-MAGV-0005", "SIM-MAGV-0002", "SIM-MAGV-0006"]
        # car_id = ["SIM-MAGV-0002", "SIM-MAGV-0006", "SIM-MAGV-0004", "SIM-MAGV-0001", "SIM-MAGV-0005"]
        car_init_pos_list = [Position(190, 436, -16), Position(185, 433, -16), Position(195, 433, -16),
                             Position(180.5, 430.5, -16), Position(199.5, 430.5, -16)]
        # car_init_pos_list = [Position(180.5, 430.5, -16), Position(199.5, 430.5, -16), Position(190, 438, -16),
        #                      Position(185, 433, -16), Position(195, 433, -16)]
        # self.car_list长度为5的car类
        for i in range(len(car_id)):
            self.car_list.append(Car(car_id[i], car_init_pos_list[i]))

    def init_car_pos(self):  # 调整小车顺序1 4 2 5 6
        car_sn_list = ["SIM-MAGV-0001", "SIM-MAGV-0004", "SIM-MAGV-0002", "SIM-MAGV-0005", "SIM-MAGV-0006"]
        car_init_pos_list = [Position(185, 433, -16), Position(190, 436, -16), Position(180.5, 430.5, -16),
                             Position(195, 433, -16),
                             Position(199.5, 430.5, -16)]

        for i in range(5):  # 1，4，2，5，6 依次移动到初始位置
            car_sn = car_sn_list[i]
            car_physical_status = next(
                (car for car in self.car_physical_status if car.sn == car_sn), None)
            car_pos = car_physical_status.pos.position
            print("current car:", car_pos)
            self.move_car_with_start_and_end(
                car_sn, car_pos, car_init_pos_list[i], 5.0, WorkState.MOVE_CAR_GO_TO_LOADING_POINT)  # 不要休眠不行
            print(self.events)

        # car_sn = "SIM-MAGV-0005"
        # cur_car_physical_status = next((car for car in self.car_physical_status if car.sn == "SIM-MAGV-0005"), None)
        # car_pos = cur_car_physical_status.pos.position
        # loading_pos = Position(self.loading_cargo_point['x'], self.loading_cargo_point['y'],
        #                        self.loading_cargo_point['z'])
        # self.move_car(car_sn,car_pos,loading_pos,CarDirection.GO)
        # rospy.sleep(1000)

    # 放在发车之前，判断是否有可飞的航线
    def judge_fly_or_not(self):
        # 遍历3个航线
        for i in range(len(self.route_state)):
            cur_route_state = self.route_state[i]
            # 首先检查环线承载能力
            if cur_route_state.cur_drone_cnt < cur_route_state.max_drone_cnt:
                # 检查放飞条件：跟上一架飞机的时间或距离达到要求
                print('(6/23)当前航线{0}未满足放飞上限,继续查询是否可飞'.format(i))
                # 计算需要的最小时间差为多少，
                '''**最小时间差需要微调 '''
                min_time = (2 * cur_route_state.height + 10) - 60  # 上升和降落的距离再加10
                cur_time = int(time.time())
                # 如果当前系统时间 减去 此环路中上一架飞机的起飞时间 大于等于最小时间差，则
                if cur_time - cur_route_state.latest_drone_sn_fly_time >= min_time:  # 判断与上一架无人机飞行的时间差是否合适
                    print('(7/24)当前航线{0}满足可飞条件'.format(i))
                    return i
        return -1

    # drone_out函数用来挂货
    def drone_out(self, car_sn, drone_sn, task_route_id):
        # 获取航线对应的订单
        waybill = self.waybill_position[task_route_id][0]
        self.waybill_position[task_route_id].pop(0)
        # 上货
        cargo_id = waybill['cargoParam']['index']
        print('(8/25)获取当前航线订单为{0}'.format(cargo_id))
        self.move_cargo_in_drone(cargo_id, drone_sn, 5.0)
        print('(9/26)绑定订单{0}成功！无人车将移动到放飞点'.format(cargo_id))
        car = next((car for car in self.car_list if car.car_sn == car_sn), None)
        car.task['drone_sn'] = drone_sn
        # car.task['route_id'] = i

        # 放飞之后更新latest_drone_sn

    '''现在理一下思路：
    day 2024 10.4 12:00
    (1)发车效率<飞机效率(包括返航)：只启用三辆车，使用三个卸货点，遇到的问题是请求返航不及时！，有小车在那等返航，然后无人机起飞了！。有无解决办法：在不加车的情况下
        智能改判断条件，但效率太低了，还不如原来吧。下下策
    (2)发车效率>飞机效率：  只启用三辆车，使用两个卸货点，遇到的问题是判断航线条件不满足，因为当前是在到loading_pos的时候判断上货，这样不行！如果改成在出发前就判断订单
        可以解决这个问题，但效率会很低，我们可以观察一下。
    (3)5辆车 和 三个卸货点 -> 我觉得可以一试 我们要保证的是发车效率和飞机效率尽量达到平衡，再次也是发车效率要大于飞机效率


    阵法的转换，圆阵可以一试， 目前完成✅急需再改
    判断上货时机改变  
    要不要指定返航飞机呢，因为不知道是否会出现航路交叉，难点在于什么，很难保证平衡，先不试了

    19:00
    平衡要有两点：小车效率、飞机效率
    目前存在的问题有
    不稳定，有两方面，(1)第一方面为飞机返航不及时（这来自于小车效率）(2)第二方面为有时候仍然存在一架飞机返航降落，一架飞机起飞的航线交叉的问题

    解决办法有
    (1)第一个问题可以将返航的第一个中间点拉斜来缓解一下
    (2)第二个问题需要划分降落的飞机吗->我觉得可行，查询发车的顺序和条件没发生变化，不知道是否会不及时返航，这就是问题1面临的了


    '''

    def circle_drone_route_test(self):
        rospy.sleep(1.0)
        six_car_physical_status = next(
            (car for car in self.car_physical_status if car.sn == "SIM-MAGV-0006"), None)
        six_car_pos = six_car_physical_status.pos.position
        while self.one_level_height_des_pos_reached(Position(0, 0, 0), six_car_pos, 0.5):
            rospy.sleep(1.0)
            six_car_physical_status = next(
                (car for car in self.car_physical_status if car.sn == "SIM-MAGV-0006"), None)
            six_car_pos = six_car_physical_status.pos.position
        '''1. 将小车移动到既定的排列位置'''
        self.init_car_pos()
        self.init_car_arr()
        '''2.将订单按送货地点进行分类'''
        self.waybill_classification()
        '''3.定义需要使用的一些变量'''
        waybill_count = 0
        # 地勤点坐标
        loading_pos = Position(self.loading_cargo_point['x'], self.loading_cargo_point['y'],
                               self.loading_cargo_point['z'])
        # 无人机既定航线
        # 航线的形状还需要再重新确定一下
        ''''146,186,-34': 0, '508,514,-22': 1, '490,390,-22': 2,'''
        drone_go_route = [
            [Position(0, 0, -64), Position(130, 300, -64), Position(146, 186, -64), Position(146, 186, -39)],
            [Position(0, 0, -74), Position(490, 500, -74), Position(508, 514, -74), Position(508, 514, -27)],
            [Position(0, 0, -110), Position(200, 400, -110), Position(490, 390, -110), Position(490, 390, -27)]]

        drone_back_route = [
            [Position(0, 0, -84), Position(210, 260, -84), Position(0, 0, -84), Position(0, 0, -21)],
            [Position(0, 0, -84), Position(0, 0, -84), Position(0, 0, -21)],
            [Position(0, 0, -116), Position(0, 0, -116), Position(0, 0, -21)]]
        # 判断无人机是否被使用过
        drone_used_or_new_list = ['new'] * 30
        last_drone_id = 0

        '''无人车相关的变量'''
        # 如果没有车正在往返地勤点（原谅我受不了这么长的名字，把你的falg干掉了）
        # 曾用名：car_moveLoadingPos_drone_bill_moveLeavingPos_flag（曾经的英姿永存）
        no_car_loading_to_leaving = True
        '''无人机相关的变量'''
        # Q：需要一个航线一个数组吗？还是全部放到一个数组中即可？
        #    如果一个航线用一个数组，那意义是什么？分航线处理的优先级？需要吗？
        cur_go_drone = []
        cur_back_drone = []
        '''辅助输出信息的变量'''
        frequency = 0

        while not rospy.is_shutdown() and waybill_count < len(self.waybill_infos):
            print('(0)当前是第{0}次循环'.format(frequency))
            print('\n***************本次执行发生的事件有:\n{0}\n***************'.format(self.events))
            '''5.检查在空飞机的状态，如是否需要返航'''
            for i in range(len(cur_go_drone) - 1, -1, -1):

                cur_drone = cur_go_drone[i]
                drone_sn = cur_drone.drone_sn
                # 获取当前飞机信息
                cur_drone_physical_status = next(
                    (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                cur_drone_pos = cur_drone_physical_status.pos.position
                print('无人机{0}的当前位置为{1},des_pos为{2}'.format(drone_sn, cur_drone_pos, cur_drone.des_pos))
                # 如果已经到卸货点，就安排卸货
                if (self.des_pos_reached(cur_drone.des_pos, cur_drone_pos, 2)
                        and cur_drone_physical_status.drone_work_state == DronePhysicalStatus.READY):
                    print('(12)无人机{0}已经到到达卸货点，进行卸货，用时1s'.format(drone_sn))

                    self.release_cargo(drone_sn, 2.0, WorkState.RELEASE_DRONE_RETURN)

                    '''***这是天上返航的效率，也是飞机效率'''
                    print('(13)无人机{0}卸货成功！查询空闲无人车'.format(drone_sn))

                    # 然后安排返航
                    # 查找空闲无人车，这边先看看情况，要是还有问题，可以固定小车
                    for j in range(len(self.car_list) - 1, -1, -1):  # 逆序遍历空闲无人车，这样后面的就先停下了
                        cur_car = self.car_list[j]
                        if cur_car.state == "empty":
                            # 获取这个卸货点的id
                            print('(14)查询空闲无人车{0}成功！查询卸货点的id以布置航线'.format(cur_car.car_sn))
                            unloading_station = next((station for station in self.unloading_cargo_stations
                                                      if station['position']['x'] == cur_drone.des_pos.x
                                                      and station['position']['y'] == cur_drone.des_pos.y
                                                      and station['position']['z'] == cur_drone.des_pos.z), None)
                            print('(15)当前卸货点的id为{0},准备返航'.format(unloading_station))
                            # 获取这个卸货点的返回航线
                            route = drone_back_route[unloading_station['index'] - 1]
                            cur_car.task['drone_sn'] = drone_sn
                            cur_car.task['route_id'] = unloading_station['index'] - 1
                            drone_physical_status = next(
                                (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                            drone_pos = drone_physical_status.pos.position
                            car_physical_status = next(
                                (car for car in self.car_physical_status if car.sn == cur_car.car_sn), None)
                            car_pos = car_physical_status.pos.position
                            # 设置返回航线
                            route[0].x = drone_pos.x
                            route[0].y = drone_pos.y + 8  # 将返航的第一个中间点向y轴方向，拉斜
                            route[-1].x = car_pos.x
                            route[-1].y = car_pos.y
                            route[-2].x = car_pos.x
                            route[-2].y = car_pos.y
                            self.fly_one_route(drone_sn, route, 15.0, 0, WorkState.MOVE_CAR_BACK_TO_LOADING_POINT)
                            cur_car.state = "bind"
                            print('(16)无人机{0}正在返航！'.format(drone_sn))
                            cur_back_drone.append(
                                Drone(drone_sn, cur_car.car_init_pos, 0, "", cur_car.car_sn))
                            cur_go_drone.pop(i)
                            break
            '''6.检查在返航中的飞机的状态，如是否已经落地，进行后续操作'''
            for i in range(len(cur_back_drone) - 1, -1, -1):
                cur_drone = cur_back_drone[i]
                # print(cur_back_drone[i])
                drone_sn = cur_drone.drone_sn  # 当前已落地的无人机
                # if drone_sn ==
                # 获取当前飞机信息
                cur_drone_physical_status = next(
                    (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                cur_drone_pos = cur_drone_physical_status.pos.position  # 当前飞机的位置
                # 获取接驳小车信息
                cur_car_sn = cur_drone.car_sn
                cur_car_physical_status = next((car for car in self.car_physical_status if car.sn == cur_car_sn), None)
                cur_car_pos = cur_car_physical_status.pos.position
                if (self.des_pos_reached(cur_drone.des_pos, cur_drone_pos, 2)
                        and cur_drone_physical_status.drone_work_state == DronePhysicalStatus.READY
                        and cur_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                    '''**这是地上小车的效率，小车效率'''
                    print('**(17)无人机{0}成功降落在无人车上{1},检查是否满足到loading_pos的条件'.format(drone_sn,
                                                                                                        cur_car_sn))
                    # 如果没有车正在往返地勤点，就可以安排回收飞机等操作
                    if no_car_loading_to_leaving:
                        # 去地勤点
                        '''去loadingpos 的时候要先查询是否有航线满足条件，有就发车，没有就停在这
                        对返航回来降落的无人车也同理啊'''
                        if self.judge_fly_or_not() >= 0:
                            route_res = self.judge_fly_or_not()
                            self.move_car(cur_car_sn, cur_car_pos, loading_pos, CarDirection.GO)
                            print('(18)锁打开中，无人车{0}携带无人机{1}前往loading_pos'.format(cur_car_sn, drone_sn))
                            # print('(18)锁打开中，无人车{0}携带无人机{1}前往loading_pos'.format(cur_car_sn, drone_sn))
                            # 记得更新状态啊啊啊啊啊
                            # car_state_list[i] = 'go'
                            cur_car = next((car for car in self.car_list if car.car_sn == cur_car_sn), None)
                            cur_car.state = "go_with_drone"
                            cur_car.task['route_id'] = route_res
                            no_car_loading_to_leaving = False
                            break
            '''7.空闲小车放飞模块'''
            # 如果没有车正在往返地勤点
            if no_car_loading_to_leaving:
                # 检索空车
                for i in range(len(self.car_list)):
                    cur_car = self.car_list[i]
                    if cur_car.state == 'empty':
                        empty_car_sn = cur_car.car_sn
                        empty_car_physical_status = next(
                            (car for car in self.car_physical_status if car.sn == empty_car_sn), None)
                        empty_car_pos = empty_car_physical_status.pos.position
                        # 判断无人车是否为ready的状态 and 在它对应的起始点
                        if empty_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY and self.des_pos_reached(
                                empty_car_pos, cur_car.car_init_pos, 0.5):
                            print('(1)检索到空闲小车{0}'.format(empty_car_sn))
                            # 去地勤点
                            if self.judge_fly_or_not() >= 0:
                                route_res = self.judge_fly_or_not()
                                self.move_car(empty_car_sn, empty_car_pos, loading_pos, CarDirection.GO)
                                print('(2)空闲小车{0}符合发车条件，正前往loaidng_pos'.format(empty_car_sn))
                                # 记得更新状态啊啊啊啊啊
                                # car_state_list[i] = 'go'
                                cur_car.state = "go"
                                cur_car.task['route_id'] = route_res
                                no_car_loading_to_leaving = False
                                break
            # 增加在空飞机状态遍历的频率
            # 去地勤点的小车
            '''对于划区域返航，'''
            for i in range(len(self.car_list)):
                cur_car = self.car_list[i]
                if cur_car.state == 'go':
                    load_car_sn = cur_car.car_sn
                    load_car_physical_status = next(
                        (car for car in self.car_physical_status if car.sn == load_car_sn), None)
                    load_car_cur_pos = load_car_physical_status.pos.position
                    # 如果已经走到地勤点
                    if (self.des_pos_reached(loading_pos, load_car_cur_pos, 0.5)
                            and load_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                        print('(3)空闲小车{0}已经到达loaing_pos,将检索无人机'.format(load_car_sn))
                        # 检索空飞机
                        found_drone = False
                        for j in range(len(drone_used_or_new_list)):
                            if drone_used_or_new_list[j] == 'new':
                                load_drone_sn = self.find_id_drone(j)
                                print('(4)无人车{0}检索new无人机{1}成功，接下来绑定无人机，用时两秒'.format(load_car_sn,
                                                                                                          load_drone_sn))
                                self.move_drone_on_car(load_car_sn, load_drone_sn, 2.0,
                                                       WorkState.MOVE_CARGO_IN_DRONE)  # 绑定无人机
                                print('(5)绑定无人机{0}成功，接下来查询航线，绑定订单'.format(load_drone_sn))
                                drone_used_or_new_list[j] = 'using'
                                found_drone = True
                                route_id = cur_car.task['route_id']
                                self.drone_out(load_car_sn, load_drone_sn, route_id)  # 选航线加上货

                                self.move_car(load_car_sn, load_car_cur_pos, cur_car.car_init_pos, CarDirection.BACK)
                                print('(10)无人车{0}携带无人机{1}前往放飞点'.format(load_car_sn, load_drone_sn))
                                # route_id = cur_car.task['route_id']
                                self.route_state[route_id].latest_drone_sn_fly_time = int(
                                    time.time())  # 这个要留在一move_car 回初始位置就可以
                                cur_car.state = 'back'
                                no_car_loading_to_leaving = True
                                break
                        if not found_drone:
                            for j in range(len(drone_used_or_new_list)):
                                if drone_used_or_new_list[j] == 'used':
                                    load_drone_sn = self.find_id_drone(j)
                                    print('【4】检索used无人机{0}成功，接下来绑定无人机，用时两秒'.format(load_drone_sn))
                                    self.move_drone_on_car(load_car_sn, load_drone_sn, 2.0,
                                                           WorkState.MOVE_CARGO_IN_DRONE)  # 绑定无人机
                                    print('【5】绑定无人机{0}成功，接下来进行换电'.format(load_drone_sn))
                                    # 多了个充电操作：
                                    self.battery_replacement(load_drone_sn, 5.0, WorkState.DRONE_RETRIEVE)
                                    print('【6】换电成功，接下来查询航线，绑定订单')
                                    drone_used_or_new_list[j] = 'using'  # 换完电池那就相当于没使用过了
                                    route_id = cur_car.task['route_id']
                                    self.drone_out(load_car_sn, load_drone_sn, route_id)  # 选航线加上货

                                    self.move_car(load_car_sn, load_car_cur_pos, cur_car.car_init_pos,
                                                  CarDirection.BACK)
                                    print('(10)无人车{0}携带无人机{1}前往放飞点'.format(load_car_sn, load_drone_sn))
                                    # route_id = cur_car.task['route_id']
                                    self.route_state[route_id].latest_drone_sn_fly_time = int(time.time())
                                    cur_car.state = 'back'
                                    no_car_loading_to_leaving = True
                                    break
                        found_drone = False

                elif cur_car.state == 'go_with_drone':
                    load_car_sn = cur_car.car_sn
                    load_car_physical_status = next(
                        (car for car in self.car_physical_status if car.sn == load_car_sn), None)
                    load_car_cur_pos = load_car_physical_status.pos.position
                    # 如果已经走到地勤点
                    if (self.des_pos_reached(loading_pos, load_car_cur_pos, 0.5)
                            and load_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                        # 先回收无人机
                        retrieve_drone_sn = cur_car.task['drone_sn']
                        retrieve_drone_physical_status = next(
                            (drone for drone in self.drone_physical_status if drone.sn == retrieve_drone_sn), None)
                        load_drone_sn = retrieve_drone_sn
                        # check remaining_capacity
                        if retrieve_drone_physical_status.remaining_capacity < 35:
                            # retrieve drone
                            print('(19)小车{0}携带无人机{1}已经到达loaing_pos,将回收无人机'.format(load_car_sn,
                                                                                                   retrieve_drone_sn))
                            self.drone_retrieve(retrieve_drone_sn, load_car_sn, 10.0,
                                                WorkState.MOVE_DRONE_ON_CAR)  # 回收使用过的无人机
                            print('(20)回收无人机成功！将检索无人机')
                            drone_used_or_new_list[int(retrieve_drone_sn[-2:]) - 1] = 'used'
                            # 检索空飞机
                            found_drone = False
                            for j in range(len(drone_used_or_new_list)):
                                if drone_used_or_new_list[j] == 'new':
                                    load_drone_sn = self.find_id_drone(j)
                                    print('(21)检索new无人机{0}成功，接下来绑定无人机，用时两秒'.format(load_drone_sn))
                                    self.move_drone_on_car(load_car_sn, load_drone_sn, 2.0,
                                                           WorkState.MOVE_CARGO_IN_DRONE)  # 绑定无人机
                                    print('(22)绑定无人机{0}成功，接下来查询航线，绑定订单'.format(load_drone_sn))
                                    drone_used_or_new_list[j] = 'using'

                                    found_drone = True
                                    break
                            if not found_drone:
                                for j in range(len(drone_used_or_new_list)):
                                    if drone_used_or_new_list[j] == 'used':
                                        load_drone_sn = self.find_id_drone(j)
                                        print('(21)检索used无人机{0}成功，接进行换电操作，用时5秒'.format(load_drone_sn))
                                        self.move_drone_on_car(load_car_sn, load_drone_sn, 2.0,
                                                               WorkState.MOVE_CARGO_IN_DRONE)  # 绑定无人机
                                        # 多了个充电操作：
                                        self.battery_replacement(load_drone_sn, 5.0, WorkState.DRONE_RETRIEVE)
                                        drone_used_or_new_list[j] = 'using'  # 换完电池那就相当于没使用过了
                                        break
                            found_drone = False
                        # common part
                        route_id = cur_car.task['route_id']
                        self.drone_out(load_car_sn, load_drone_sn, route_id)  # 选航线加上货
                        self.move_car(load_car_sn, load_car_cur_pos, cur_car.car_init_pos, CarDirection.BACK)
                        print('(27)无人车{0}携带无人机{1}前往放飞点'.format(load_car_sn, load_drone_sn))
                        # route_id = cur_car.task['route_id']
                        self.route_state[route_id].latest_drone_sn_fly_time = int(time.time())
                        cur_car.state = 'back'
                        no_car_loading_to_leaving = True
            # 增加在空飞机状态遍历的频率
            # 从地勤点返回初始位置的小车
            for i in range(len(self.car_list)):
                cur_car = self.car_list[i]
                if cur_car.state == 'back':
                    fly_car_sn = cur_car.car_sn
                    fly_car_physical_status = next(
                        (car for car in self.car_physical_status if car.sn == fly_car_sn), None)
                    fly_car_cur_pos = fly_car_physical_status.pos.position
                    fly_car_init_pos = cur_car.car_init_pos
                    fly_drone_sn = cur_car.task['drone_sn']

                    route_id = cur_car.task['route_id']
                    #  print('?????????', route_id)
                    # 如果已经走到初始位置
                    if self.des_pos_reached(fly_car_cur_pos, fly_car_init_pos,
                                            0.5) and fly_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                        print('(11)无人车{0}携带无人机{1}到达放飞点，准备进行放飞'.format(fly_car_sn, fly_drone_sn))
                        # 放飞
                        route = drone_go_route[route_id]
                        route[0].x = fly_car_cur_pos.x
                        route[0].y = fly_car_cur_pos.y
                        self.fly_one_route(fly_drone_sn, route, 15.0, 0, WorkState.RELEASE_CARGO)
                        print('**放飞无人机{0}的时间为{1}'.format(fly_drone_sn, int(time.time())))
                        print('(12)无人机{0}放飞成功！'.format(fly_drone_sn))
                        # # 需要更新放飞当前无人机的系统时间为多少
                        # self.route_state[route_id].latest_drone_sn_fly_time = int(time.time())
                        unloading_station = next(
                            (station for station in self.unloading_cargo_stations
                             if station['index'] == route_id + 1), None)
                        des_pos = Position(unloading_station['position']['x'],
                                           unloading_station['position']['y'],
                                           unloading_station['position']['z'])
                        cur_go_drone.append(
                            Drone(fly_drone_sn, des_pos, 0, fly_car_sn))
                        cur_car.state = 'empty'

            '''一次循环结束'''
            rospy.sleep(1.0)
            frequency += 1  # 循环数加1

        '''Game Over'''
        print(
            'Total waybill finished: ',
            waybill_count,
            ', Total score: ',
            self.score)


if __name__ == '__main__':
    race_demo = DemoPipeline()
    race_demo.circle_drone_route_test()
    # race_demo.hello()

