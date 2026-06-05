import os
import sys
import threading
# now_path = 'F://CARLA_0.9.15//WindowsNoEditor//PythonAPI//carla'
# sys.path.append(now_path)
import carla
import time
import math
import numpy as np

from agents.navigation.my_global_route_planner import NpcGlobalRoutePlanner  # 如果是behavior_agent的话需要用这个
# from agents.navigation.global_route_planner import GlobalRoutePlanner

from agents.navigation.controller import VehiclePIDController, PIDLongitudinalController
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.tools.misc import draw_waypoints, distance_vehicle, vector, is_within_distance, get_speed
from risk2scenario.core.statement import Statement
from risk2scenario.core.testcase import TestCase

has_collided = False


def collision_callback (event):
    global has_collided
    if not has_collided:
        has_collided = True
        print("----------------------------COLLISION----------------------------")


def calcu_offset (id_1, id_2, offset_1, offset_2):
    """
    计算两个对象的 ID 偏移量和距离偏移量。

    :param id_1: 第一个对象的 ID。
    :param id_2: 第二个对象的 ID。
    :param offset_1: 第一个对象的偏移量。
    :param offset_2: 第二个对象的偏移量。
    :return: (id_offset, dis_offset)
    """
    id_offset = id_1 - id_2
    dis_offset = offset_1 - offset_2
    print("id_offset: ", id_offset)
    print("dis_offset: ", dis_offset)
    return id_offset, dis_offset


class CarlaWorld:
    def __init__ (self):
        self.lane_follow_in_progress = {}
        self.ego_spawn_point = None
        self.ego_location = None
        self.ego_vehicle = None
        self.agent = None
        self.npc_list = {}
        self.lane_change_in_progress = {}  # 初始化为一个空字典
        # self.lane_change_in_progress = False
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(30.0)  # 将超时设置为 10 秒
        # self.world = self.client.load_world('Town06')
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        # 碰撞检测
        self.collision_sensor_bp = self.world.get_blueprint_library().find('sensor.other.collision')
        # 开启同步模式
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)
        self.world.tick()

    def check_connection (self):
        """验证连接是否有效"""
        try:
            # 尝试获取一个简单属性来检测连接状态
            _ = self.world.get_settings()
            return True
        except RuntimeError:
            return False

    def soft_reset (self):
        """温和重置连接"""
        try:
            # 重新初始化关键连接参数
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)  # 适当缩短超时
            self.world = self.client.get_world()
            print("连接已软重置")
            return True
        except Exception as e:
            print(f"软重置失败: {str(e)}")
            return False

    def load_map_needed (self, map_name):
        # current_map = self.world.get_map().name
        current_map = os.path.basename(self.world.get_map().name)  # 或者使用 split() 方法
        print(f"Current map: {current_map}")
        if current_map != map_name:
            print(f"Loading map {map_name}...")
            self.world = self.client.load_world(map_name)
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            self.world.apply_settings(settings)
            self.map = self.world.get_map()
        else:
            print(f"Map {map_name} is already loaded.")

    def set_weather (self, world):
        # 创建自定义天气参数
        weather = carla.WeatherParameters(
            cloudiness=80,  # 云量，范围 0-100
            precipitation=0,  # 降雨量，范围 0-100
            precipitation_deposits=20,  # 积水量，范围 0-100
            wind_intensity=0,  # 风强度，范围 0-100
            fog_density=0,  # 雾密度，范围 0-100
            wetness=0.0,  # 湿度，范围 0-100
            sun_azimuth_angle=0.0,  # 太阳方位角，范围 0-360
            sun_altitude_angle=20.0  # 太阳仰角，范围 -90 到 90
        )
        world.set_weather(weather)

    def spawn_ego_by_point (self, actor_spawn_point):
        vehicle_bp = self.world.get_blueprint_library().filter('model3') [0]
        vehicle_bp.set_attribute('color', '255,0,0')  # 设置主车颜色为红色 (RGB: 255,0,0)
        ego_vehicle = self.world.try_spawn_actor(vehicle_bp, actor_spawn_point)
        print(f"Egoo vehicle spawned at {actor_spawn_point.location}")
        return ego_vehicle

    def draw_target_line (self, waypoints):
        debug = self.world.debug
        life_time = 10
        color = carla.Color(255, 45, 0)
        thickness = 0.3
        for i in range(len(waypoints) - 1):
            # 输出每一个点的坐标
            # print("`````````````````````````````````````````````````````````")
            # print(f"({waypoints[i][0].transform.location.x}, {waypoints[i][0].transform.location.y})")
            debug.draw_line(waypoints [i] [0].transform.location + carla.Location(z=0.5),
                            waypoints [i + 1] [0].transform.location + carla.Location(z=0.5),
                            thickness=thickness, color=color, life_time=life_time)

    def draw_current_point (self, current_point):
        self.world.debug.draw_point(current_point, size=0.1, color=carla.Color(b=255), life_time=25)

    def set_spectator (self, vehicle):
        self.world.get_spectator().set_transform(
            carla.Transform(vehicle.get_transform().location +
                            carla.Location(z=30), carla.Rotation(pitch=-90))
        )

    def get_spawn_point (self, target_road_id, target_lane_id):
        # 获取生成点
        waypoints = self.map.generate_waypoints(5.0)
        for waypoint in waypoints:
            if waypoint.road_id == target_road_id:
                lane_id = waypoint.lane_id
                if lane_id == target_lane_id:
                    location = waypoint.transform.location
                    location.z = 1
                    the_spawn_point = carla.Transform(location, waypoint.transform.rotation)
                    break
        return the_spawn_point

    def init_ego (self, testcase, agent_type='basic', behavior='cautious'):
        ego_statement = None

        for statement in testcase.constructor_statements:
            if statement.assignee == 'ego':
                ego_statement = statement
                break

        # 在合适的位置生成主车，目前暂时是一个固定位置
        lane_id = ego_statement.args.get('lane_id', 5)
        base_y = 237.568939
        y_position = base_y + (5 - lane_id) * 3.5
        start_location = carla.Location(x=100, y=y_position, z=1.0)
        start_point = carla.Transform(start_location, carla.Rotation(yaw=0))
        start_waypoint = self.world.get_map().get_waypoint(start_location)
        forward_vec = start_point.rotation.get_forward_vector()

        self.ego_vehicle = self.spawn_ego_by_point(start_point)
        self.ego_spawn_point = start_point
        self.world.tick()
        if self.ego_vehicle is not None:
            print(f"Ego vehicle spawned at {start_location}")
        else:
            print("Ego vehicle spawn failed.")
            return None

        initial_speed = ego_statement.args.get('initial_speed', 20)  # 默认速度为 20 km/h
        if initial_speed > 0:
            initial_speed_ms = initial_speed * (1000 / 3600)
            self.ego_vehicle.set_target_velocity(carla.Vector3D(x=initial_speed_ms, y=0, z=0))
            print(f"Ego vehicle initial speed set to {initial_speed} km/h")

        # 设置主车的代理
        self.world.tick()
        next_waypoint = start_waypoint.next(300) [0]

        if agent_type == 'basic':
            # 使用 BasicAgent
            self.agent = BasicAgent(self.ego_vehicle)
            self.agent.set_target_speed(initial_speed)
            self.agent.set_destination(next_waypoint.transform.location)
            print(f"Ego vehicle destination set to {next_waypoint.transform.location}")
        elif agent_type == 'behavior':
            # 使用 BehaviorAgent
            self.agent = BehaviorAgent(self.ego_vehicle, behavior=behavior)
            self.agent.set_destination(start_location, next_waypoint.transform.location)
            print(f"Ego vehicle destination set to {next_waypoint.transform.location}")
        else:
            print(f"Unknown agent type: {agent_type}")
            return None

    def init_npc (self, testcase):
        """
        初始化 NPC 车辆，根据主车的位置，在相对位置上生成 NPC 车辆，并设置初始速度
        :param testcase: 包含 NPC 车辆参数的测试用例
        """
        # 初始化主车参数
        ego_lane_id = None
        ego_offset = None

        # 首先找到主车的构造语句并提取参数
        for statement in testcase.constructor_statements:
            if statement.assignee == 'ego':
                # 获取主车的参数，默认值为0
                ego_lane_id = statement.args.get('lane_id', 0)
                ego_offset = statement.args.get('offset', 0)
                print(f"Ego vehicle lane_id: {ego_lane_id}, offset: {ego_offset}")
                break  # 找到主车构造语句后退出循环

        # 如果没有找到主车的构造语句，抛出异常
        if ego_lane_id is None or ego_offset is None:
            raise ValueError("Ego vehicle parameters are not initialized.")

        # 获取主车的位置
        if self.ego_vehicle is None:
            raise ValueError("Ego vehicle is not initialized.")
        ego_location = self.ego_location  # 假设 self.ego_location 已经正确初始化
        print(f"Ego vehicle location: {ego_location}")

        # 处理 NPC 车辆的构造语句
        for statement in testcase.constructor_statements:
            if statement.assignee != 'ego':
                # 获取 NPC 车辆的参数
                npc_lane_id = statement.args.get('lane_id', 0)
                npc_offset = statement.args.get('offset', 0)
                npc_initial_speed = statement.args.get('initial_speed', 0)
                print(f"NPC vehicle lane_id: {npc_lane_id}, offset: {npc_offset}, initial_speed: {npc_initial_speed}")
                # 计算 NPC 车辆的偏移量
                id_offset, dis_offset = calcu_offset(npc_lane_id, ego_lane_id, npc_offset, ego_offset)
                # 生成 NPC 车辆
                npc_transform = self.spawn_npc_by_lane_offset(self.ego_spawn_point, id_offset, dis_offset)
                self.world.tick()
                if npc_transform:
                    npc_vehicle = self.spawn_npc_by_point(npc_transform)
                    print(f"NPC vehicle {npc_vehicle.id} spawned at {npc_transform.location}")
                    npc_initial_speed_ms = npc_initial_speed * (1000 / 3600)  # 将 km/h 转换为 m/s
                    npc_vehicle.set_target_velocity(carla.Vector3D(x=npc_initial_speed_ms, y=0, z=0))
                    print(f"NPC vehicle {npc_vehicle.id} initial speed set to {npc_initial_speed} km/h")
                    self.lane_change_in_progress [npc_vehicle.id] = False
                    self.lane_follow_in_progress [npc_vehicle.id] = False

                    # 存储 NPC 车辆信息
                    self.npc_list [statement.assignee] = {
                        "vehicle_id": npc_vehicle.id,
                        "lane_id": npc_lane_id,
                        "offset": npc_offset,
                        "initial_speed": npc_initial_speed
                    }
                    print(f"NPC vehicle spawned: {statement.assignee} at lane {npc_lane_id}, offset {npc_offset}")

    def active_npc_action (self, npc_vehicle, action_name, args, testcase):
        npc_vehicle.disable_constant_velocity()
        args_lateral_dict = {'K_P': 0.9, 'K_D': 1.2, 'K_I': 0.2, 'dt': 1.0 / 10.0}
        args_long_dict = {'K_P': 0.5, 'K_D': 0.0, 'K_I': 0.75, 'dt': 1.0 / 10.0}
        pid_controller = VehiclePIDController(npc_vehicle, args_lateral_dict, args_long_dict)
        if action_name == "accelerate" or action_name == "decelerate":
            if args ["target_speed"] == 0.0:
                npc_vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1))
            else:
                # direction = None
                # yield from self.lane_follow(npc_vehicle, direction, target_speed=args["target_speed"],
                #                             pid_controller=pid_controller)
                ackermann_control = carla.VehicleAckermannControl()
                ackermann_control.acceleration = -3.0 if action_name == "decelerate" else 2.0
                ackermann_control.speed = args ["target_speed"] * (1000 / 3600)
                print(f"NPC vehicle{npc_vehicle.id} {action_name} to {args ['target_speed']} km/h")
                print("NPC vehicle apply ackermann control")
                npc_vehicle.apply_ackermann_control(ackermann_control)
                yield
                # npc_vehicle.set_target_velocity(carla.Vector3D(x=args["target_speed"] * (1000 / 3600), y=0, z=0))
            print(f"NPC vehicle{npc_vehicle.id} target speed set to {args ['target_speed']} km/h")

        if action_name == "changeLane":
            target_lane_id = args ["target_lane"]
            target_speed = args.get("target_speed", 20)
            current_lane_id = None
            for statement in testcase.constructor_statements:
                if statement.assignee != 'ego' and self.npc_list [statement.assignee] ["vehicle_id"] == npc_vehicle.id:
                    npc_assignee = statement.assignee
                    current_lane_id = self.npc_list [npc_assignee] ["lane_id"]
                    print(f"pre list NPC vehicle{npc_vehicle.id} current lane_id: {current_lane_id}")
                    break

            previous_target_lane_id = self.find_previous_target_lane(testcase, npc_assignee, args ["trigger_sequence"])
            if npc_assignee and previous_target_lane_id is not None:
                self.npc_list [npc_assignee] ["lane_id"] = previous_target_lane_id
                print(f"now list npc_list: {self.npc_list}")

                print(f"NPC vehicle{npc_vehicle.id} lane_id set to {previous_target_lane_id}")
                current_lane_id = previous_target_lane_id

            else:
                print(f"NPC vehicle{npc_vehicle.id} is not initialized.")

            yield from self.execute_lane_changes(npc_vehicle, current_lane_id, target_lane_id, target_speed,
                                                 pid_controller)

    def find_previous_target_lane (self, testcase, npc_assignee, current_trigger_sequence):
        """
        查找离当前 trigger_sequence 最近的上一个 changeLane 语句的目标车道
        """
        previous_target_lane_id = None
        max_trigger_sequence = -1  # 用于记录离当前 trigger_sequence 最近的上一个 trigger_sequence

        for statement in testcase.method_statements:
            print(f"Statement: {statement.callee}, {statement.method_name}, {statement.args}")
            if (statement.callee == npc_assignee and
                    statement.method_name == "changeLane" and
                    statement.args ["trigger_sequence"] < current_trigger_sequence):
                # 如果当前语句的 trigger_sequence 比之前记录的最大值更大，则更新
                if statement.args ["trigger_sequence"] > max_trigger_sequence:
                    max_trigger_sequence = statement.args ["trigger_sequence"]
                    previous_target_lane_id = statement.args ["target_lane"]
        print(f"Previous target lane ID: {previous_target_lane_id}")
        return previous_target_lane_id

    def execute_lane_changes (self, npc_vehicle, current_lane_id, target_lane_id, target_speed, pid_controller=None):
        #
        print(f"Current lane ID: {current_lane_id}, Target lane ID: {target_lane_id}")
        # 计算需要跨越的车道数量
        lane_num = abs(target_lane_id - current_lane_id)
        print(f"Lane change distance: {lane_num} lanes")
        for _ in range(lane_num):
            direction = "left" if current_lane_id < target_lane_id else "right"
            yield from self.lane_change_direction(npc_vehicle, direction, target_speed=target_speed,
                                                  pid_controller=pid_controller)
            # 更新当前车道 ID
            current_lane_id += 1 if direction == "left" else -1
            print(f"Current lane ID: {current_lane_id}")
        yield

    # def spawn_actor_by_point(self, ego_spawn_point):
    #     vehicle_bp = self.world.get_blueprint_library().filter('vehicle.tesla.*')[0]
    #     ego_vehicle = self.world.try_spawn_actor(vehicle_bp, ego_spawn_point)
    #
    #     return ego_vehicle

    def spawn_ego_by_point (self, actor_spawn_point):
        vehicle_bp = self.world.get_blueprint_library().filter('model3') [0]
        vehicle_bp.set_attribute('color', '255,0,0')  # 设置主车颜色为红色 (RGB: 255,0,0)
        actor = self.world.try_spawn_actor(vehicle_bp, actor_spawn_point)
        return actor

    def spawn_npc_by_point (self, actor_spawn_point):
        vehicle_bp = self.world.get_blueprint_library().filter('model3') [0]
        vehicle_bp.set_attribute('color', '0,0,255')  # 设置npc车颜色为蓝色 (RGB: 0,0,255)
        actor = self.world.try_spawn_actor(vehicle_bp, actor_spawn_point)
        return actor

    def spawn_npc_by_lane_offset (self, ego_spawn_point, id_offset, dis_offset):
        rotation = ego_spawn_point.rotation
        location = ego_spawn_point.location
        waypoint = self.map.get_waypoint(location)

        forward_vector = ego_spawn_point.rotation.get_forward_vector()
        # 越往左lane_id越大
        if id_offset == 0:
            pre_target_waypoint = waypoint
            if dis_offset > 0:
                target_waypoint = pre_target_waypoint.next(dis_offset) [0]
                target_location = target_waypoint.transform.location
            else:
                target_waypoint = pre_target_waypoint.previous(abs(dis_offset)) [0]
                target_location = target_waypoint.transform.location

        else:
            for n in range(abs(id_offset)):
                if id_offset < 0:
                    pre_target_waypoint = waypoint.get_right_lane()
                elif id_offset > 0:
                    pre_target_waypoint = waypoint.get_left_lane()
                if not pre_target_waypoint:
                    print(f"Unable to find target lane with id_offset={id_offset}.")
                    return None
                waypoint = pre_target_waypoint

        target_location = pre_target_waypoint.transform.location + carla.Location(x=forward_vector.x * dis_offset,
                                                                                  y=forward_vector.y * dis_offset, z=1)
        print(f"ego vehicle location: {location}")
        print(f"Calculated NPC vehicle location: {target_location}")

        npc_transform = carla.Transform(target_location, rotation)
        return npc_transform

    def cal_target_route (self, vehicle=None, lanechange="left", target_dis=25):
        grp = NpcGlobalRoutePlanner(self.map, 2)
        # grp = GlobalRoutePlanner(self.map, 2)
        current_location = vehicle.get_transform().location
        current_waypoint = self.map.get_waypoint(current_location)
        target_org_waypoint = current_waypoint  # 默认直行
        if lanechange in ["left", "right"]:
            temp_waypoint = (
                current_waypoint.get_left_lane()
                if lanechange == "left"
                else current_waypoint.get_right_lane()
            )
            if temp_waypoint is not None:
                target_org_waypoint = temp_waypoint
            else:
                print(f"Cannot change lane to {lanechange}, keeping current lane.")
        next_waypoints = target_org_waypoint.next(target_dis)
        if not next_waypoints:
            print(f"No waypoints {target_dis}m ahead, route calculation failed.")
            return []
        target_waypoint = next_waypoints [0]
        target_location = target_waypoint.transform.location
        print(f"Target location: {target_location}")
        route = grp.trace_route(current_location, target_location)
        return route

    def lane_follow (self, npc_vehicle, direction, target_speed, pid_controller=None):
        """加减速控制"""
        vehicle_id = npc_vehicle.id
        if self.lane_follow_in_progress.get(vehicle_id, True):
            print(f"Lane follow already in progress for NPC {vehicle_id}. Skipping action.")
            return
        self.lane_follow_in_progress [vehicle_id] = True
        print(f"Starting lane follow for NPC {npc_vehicle.id}...")
        # 获取当前位置信息
        # current_location = npc_vehicle.get_transform().location
        # current_waypoint = self.map.get_waypoint(current_location)
        route = self.cal_target_route(vehicle=npc_vehicle, lanechange=direction, target_dis=90)
        self.draw_target_line(route)
        waypoint_index = 0
        arrive_target_point = False
        target_distance_threshold = (target_speed * 1 / 3.6) * 1.1  # 转换为km/h
        start_time = self.world.get_snapshot().timestamp.elapsed_seconds
        print(f"Start time: {start_time}")

        while not arrive_target_point:
            current_time = self.world.get_snapshot().timestamp.elapsed_seconds
            if current_time - start_time > 4.95:
                print(f"Time limit exceeded for NPC {npc_vehicle.id}.Stopping action.")
                self.lane_follow_in_progress [npc_vehicle.id] = False
                break
            if waypoint_index < len(route):
                target_waypoint = route [waypoint_index] [0]
                transform = npc_vehicle.get_transform()
                self.world.debug.draw_point(transform.location, size=0.1, color=carla.Color(b=255), life_time=25)

                distance_to_waypoint = distance_vehicle(target_waypoint, transform)
                if distance_to_waypoint < target_distance_threshold:
                    waypoint_index += 1
                    if waypoint_index >= len(route):
                        arrive_target_point = True
                        print(f"NPC {npc_vehicle.id} has arrived at the target point.")
                        break
                else:
                    control = pid_controller.run_step(target_speed, target_waypoint)
                    npc_vehicle.apply_control(control)
                    npc_speed = get_speed(npc_vehicle)
                    # npc_speed = npc_vehicle.get_velocity()
                    angular_velocity = npc_vehicle.get_angular_velocity()
                    yaw_rate = angular_velocity.z
                    print(
                        f"NPC {npc_vehicle.id}  speed: {npc_speed} km/h")
            else:
                break
            yield
        self.lane_follow_in_progress [npc_vehicle.id] = False

    def lane_change_direction (self, npc_vehicle, direction, target_speed, pid_controller=None):
        """异步执行换道操作"""
        vehicle_id = npc_vehicle.id
        if self.lane_change_in_progress.get(vehicle_id, True):
            print(f"Lane change already in progress for NPC {vehicle_id}. Skipping action.")
            return
        # 标记为正在换道
        self.lane_change_in_progress [vehicle_id] = True
        print(f"Starting lane change for NPC {npc_vehicle.id}...")
        # 获取当前位置信息
        # current_location = npc_vehicle.get_transform().location
        # current_waypoint = self.map.get_waypoint(current_location)
        route = self.cal_target_route(vehicle=npc_vehicle, lanechange=direction, target_dis=max(target_speed * 1.3, 20))
        # route = self.cal_target_route(vehicle=npc_vehicle, lanechange=direction, target_dis=20)

        print(f"target_dis: {target_speed * 1.5}m")
        self.draw_target_line(route)

        waypoint_index = 0
        arrive_target_point = False
        target_distance_threshold = (target_speed * 1 / 3.6) * 0.6  # 转换为km/h

        while not arrive_target_point:
            if waypoint_index < len(route):
                target_waypoint = route [waypoint_index] [0]
                transform = npc_vehicle.get_transform()
                self.world.debug.draw_point(transform.location, size=0.1, color=carla.Color(b=255), life_time=25)

                distance_to_waypoint = distance_vehicle(target_waypoint, transform)
                if distance_to_waypoint < target_distance_threshold:
                    waypoint_index += 1
                    if waypoint_index >= len(route):
                        arrive_target_point = True
                        print(f"NPC {npc_vehicle.id} has arrived at the target point.")
                        break
                else:
                    control = pid_controller.run_step(target_speed, target_waypoint)
                    npc_vehicle.apply_control(control)
                    npc_speed = get_speed(npc_vehicle)
                    # npc_speed = npc_vehicle.get_velocity()
                    angular_velocity = npc_vehicle.get_angular_velocity()
                    yaw_rate = angular_velocity.z
                    print(
                        f"NPC {npc_vehicle.id}  speed: {npc_speed} km/h")
            else:
                break
            yield
        # 一旦换道完成
        npc_location = npc_vehicle.get_transform().location
        print(f"NPC {npc_vehicle.id} has stopped at {npc_location}.")
        npc_waypoint = carla.Transform(npc_location, carla.Rotation(yaw=0))
        # x方向的速度为npc_speed，y方向的速度为0，z方向的速度为0
        npc_forward_vec = npc_waypoint.rotation.get_forward_vector()
        # npc_vehicle.set_target_velocity(npc_forward_vec * npc_speed * (1000 / 3600))

        npc_vehicle.enable_constant_velocity(carla.Vector3D(x=npc_speed * (1000 / 3600), y=0, z=0))  # 启用持续速度

        # npc_vehicle.set_target_velocity(carla.Vector3D(x=npc_speed * (1000 / 3600), y=0, z=0))

        print(f"NPC {npc_vehicle.id} target speed set to {npc_speed} km/h")
        # 输出车辆的控制信息
        #
        # npc_vehicle.set_velocity(carla.Vector3D(x=0, y=0, z=0))
        npc_vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=0))
        # print(f"NPC {npc_vehicle.id} has stopped.")
        self.lane_change_in_progress [npc_vehicle.id] = False
