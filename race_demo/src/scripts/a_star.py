#!/usr/bin/env python3
import heapq
import math
import numpy as np
import pymtmap
from race_demo.srv import QueryVoxel, QueryVoxelRequest

# 定义节点类,用于A*路径规划
class Node:
    def __init__(self, x, y, z, parent=None):
        self.x = x
        self.y = y
        self.z = z
        self.g = 0  # 从起点到当前节点的代价
        self.h = 0  # 启发式代价（当前节点到目标节点的估计代价）
        self.f = 0  # 总代价 f = g + h
        self.parent = parent  # 用于路径回溯

    def __lt__(self, other):
        return self.f < other.f


# 启发式函数（使用欧几里得距离）
def heuristic(node, goal):
    return math.sqrt((node.x - goal[0]) ** 2 + (node.y - goal[1]) ** 2)

# 获取邻居节点
def get_neighbors(node, step_size, threshold):
    neighbors = []
    for dx in [-step_size, 0, step_size]:
        for dy in [-step_size, 0, step_size]:
            if dx == 0 and dy == 0:
                continue  # 忽略自身
            new_x, new_y = node.x + dx, node.y + dy
            if query_distance_to_obstacle(new_x, new_y, node.z) > threshold:
                neighbors.append(Node(new_x, new_y, node.z, parent=node))
    return neighbors

def query_distance_to_obstacle(x,y,z):
    voxel = map_instance.Query(x, y, z)
    distance_to_obstacle = voxel.distance
    return distance_to_obstacle

# A*路径规划算法
def astar(start, goal, step_size, threshold, real_start):
    open_list = []
    closed_set = set()

    start_node = Node(*start)
    goal_node = Node(*goal)

    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        found_obstacle, _ = query_nearest_obstacle_on_path(current_node, goal, step_size, threshold)
        # 如果到达目标,回溯路径
        if not found_obstacle:
            # print("回溯路径")
            path = []
            while current_node:
                path.append((current_node.x, current_node.y, current_node.z))
                current_node = current_node.parent
            if len(path) <= 1:
                # print("已经很近了")
                return path[::-1]  # 返回从起点到终点的路径
            for point in path:
                # print("寻找更近的点")
                found_obstacle, _ = query_nearest_obstacle_on_path(real_start, point, step_size, 4.5)
                if not found_obstacle:
                    # 找到第一个没有障碍物的点，获取其索引
                    index = path.index(point)
                    # 从该点之后的所有元素删除
                    path = path[:index+1]
                    # print("找到了更近的点")
                    return path[::-1]  # 返回从起点到终点的路径
            # print("没有改进")
            return path[::-1]  # 返回从起点到终点的路径
            
        closed_set.add((current_node.x, current_node.y, current_node.z))

        # 扩展邻居节点
        for neighbor in get_neighbors(current_node, step_size, threshold):
            distance_to_obstacle = query_distance_to_obstacle(neighbor.x, neighbor.y, neighbor.z)
            # print(f"扩展邻居{neighbor.x}, {neighbor.y}, {neighbor.z}: 距离障碍物{distance_to_obstacle}")
            neighbor.g = current_node.g + 1
            neighbor.h = heuristic(neighbor, goal)
            neighbor.f = neighbor.g + neighbor.h

            if not any(n.x == neighbor.x and n.y == neighbor.y for n in open_list):
                heapq.heappush(open_list, neighbor)


    return []  # 无法找到路径

# 查询直线路径上的体素距离障碍物的最近距离，返回距离小于3米的第一个体素
def query_nearest_obstacle_on_path(start, goal, step_size, threshold):
    # 如果 start 和 goal 是 Node 对象，提取它们的坐标
    if isinstance(start, Node):
        start = (start.x, start.y, start.z)
    if isinstance(goal, Node):
        goal = (goal.x, goal.y, goal.z)
    direction = np.array(goal) - np.array(start)
    direction_norm = np.linalg.norm(direction)
    unit_direction = direction / direction_norm if direction_norm != 0 else direction
    current_position = np.array(start)
     # 查询当前位置到最近障碍物的距离
    current_distance_to_obstacle = query_distance_to_obstacle(current_position[0], current_position[1], current_position[2])
    # print(f"路径起点:{current_position}, 距离障碍物:{current_distance_to_obstacle}")

    while np.linalg.norm(np.array(goal) - current_position) > current_distance_to_obstacle-threshold:
        # 计算下一个位置
        move_distance = current_distance_to_obstacle - threshold
        next_position = current_position + move_distance * unit_direction
        next_distance_to_obstacle = query_distance_to_obstacle(next_position[0], next_position[1], next_position[2])
        # print(f"移动到点: {next_position}, 距离障碍物:{next_distance_to_obstacle}")
        # 检查下下一个位置障碍物距离是否小于安全距离
        if next_distance_to_obstacle <= threshold:
            found_obstacle = True
            # print(f"危险点: {next_position}, 距离障碍物 {next_distance_to_obstacle}")
            return found_obstacle, next_position  # 找到符合条件的点，返回
        
        # 更新当前位置
        current_position = next_position
        current_distance_to_obstacle = next_distance_to_obstacle

    # 如果没有找到小于 3 米的点，则返回 None
    found_obstacle = False
    # print(f"未找到距离障碍物小于 {threshold} 米的点")
    return found_obstacle, current_position

# 主控制函数
def navigate_with_astar(start, goal, step_size, threshold):
    global map_instance
    map_file_path = "voxel_map_final.bin"
    map_instance = pymtmap.Map(map_file_path)
    print(f"规划路线:初始位置{start}->目标位置{goal}")
    found_obstacle, current_position = query_nearest_obstacle_on_path(start, goal, step_size, threshold)
    path = []
    if found_obstacle:
        # print(f"在 {current_position} 启用A*算法避障...")
        path = astar(current_position, goal, step_size, threshold, start)
        if path:
            print(f"绕过障碍物的路径: {path}")
        else:
            print("A* 无法找到路径！")
    else:
        current_position = goal
        print(f"沿直线移动到 {current_position}")

    route = [start] + path + [goal]
    print(f"最终路径: {route}")
    return route

# if __name__ == '__main__':

#     start = (184.12,433.83, -65)
#     goal = (430, 184, -65)

#     route = navigate_with_astar(start, goal, step_size=5.0, threshold=5.0)
#     print(f"路线: {route}")
