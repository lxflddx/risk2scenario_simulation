import os
import sys
import threading
#
now_path = 'F://CARLA_0.9.15//WindowsNoEditor//PythonAPI//carla'
sys.path.append(now_path)
import carla
import json
import time
import math
import numpy as np

# from agents.navigation.my_global_route_planner import NpcGlobalRoutePlanner   # 如果是behavior_agent的话需要用这个
from agents.navigation.global_route_planner import GlobalRoutePlanner  # 如果是basic_agent的话需要用这个

from agents.navigation.controller import VehiclePIDController, PIDLongitudinalController
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.tools.misc import draw_waypoints, distance_vehicle, vector, is_within_distance, get_speed
from statement import Statement
from testcase import TestCase

has_collided = False

ENV_CONFIG_SCHEMA = {
    "RoadCurvature": {
        "Straight": "Straight Road",
        "Curve": "Curved Road"
    },
    "RoadGrade": {
        "Flat": "Level Road",
        "Downhill": "Downhill",
        "Uphill": "Uphill"
    },
    "RoadFrictionCoefficient": {
        "Dry": "Dry",
        "Slippery": "Slippery"
    },
    "VisibilityConditions": {
        "Good": "Good",
        "Nighttime": "Nighttime",
        "AdverseWeather": "Adverse Weather (Rain/Fog)"
    }
}


def collision_callback (event):
    global has_collided
    if not has_collided:
        has_collided = True
        print("----------------------------COLLISION----------------------------")


class ActionException(Exception):
    def __init__ (self, message):
        super().__init__(message)


def calcu_offset(id_1, id_2, offset_1, offset_2):
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

def get_next_lane_change_interval (testcase, npc_id, current_sequence):
    """获取同一车辆的下一个换道动作的time_interval"""
    next_interval = 5  # 默认3秒
    # 查找同一车辆的下一个换道动作
    for stmt in testcase.method_statements:
        if (stmt.callee == npc_id and
                stmt.method_name == "changeLane" and
                stmt.args ["trigger_sequence"] > current_sequence):
            next_interval = stmt.args.get("time_interval", 5)
            break

    print(f"Next lane change interval for NPC {npc_id}: {next_interval}s")
    return next_interval


class CarlaWorld:
    def __init__ (self):
        self.lane_follow_in_progress = {}
        self.ego_spawn_point = None
        self.ego_location = None
        self.ego_vehicle = None
        self.agent = None
        self.npc_list = {}
        self.lane_change_in_progress = {}  # 初始化为一个空字典
        # self.active_actions = {}
        # self.lane_change_in_progress = False
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(30.0)  # 将超时设置为 10 秒
        # self.world = self.client.load_world('Town06')
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        # 碰撞检测
        self.collision_sensor_bp = self.world.get_blueprint_library().find('sensor.other.collision')
        self.friction_trigger = None
        self.npc_follow_lane_generators = {}
        self.npc_target_speeds = {}
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

    def normalize_env_config (self, env_config):
        if isinstance(env_config, str):
            env_config = json.loads(env_config)

        if env_config is None:
            env_config = {}

        normalized = {
            "RoadCurvature": env_config.get("RoadCurvature", "Straight"),
            "RoadGrade": env_config.get("RoadGrade", "Flat"),
            "RoadFrictionCoefficient": env_config.get("RoadFrictionCoefficient", "Dry"),
            "VisibilityConditions": env_config.get("VisibilityConditions", "Good")
        }

        for field_name, field_value in normalized.items():
            allowed_values = ENV_CONFIG_SCHEMA [field_name].keys()
            if field_value not in allowed_values:
                raise ValueError(
                    f"Invalid value for {field_name}: {field_value}. "
                    f"Allowed values: {list(allowed_values)}"
                )

        return normalized

    def resolve_env_config (self, env_config):
        normalized = self.normalize_env_config(env_config)

        road_curvature = normalized ["RoadCurvature"]
        road_grade = normalized ["RoadGrade"]
        friction = normalized ["RoadFrictionCoefficient"]
        visibility = normalized ["VisibilityConditions"]

        if road_grade in ["Downhill", "Uphill"]:
            target_town = "Town04"
        else:
            target_town = "Town06"

        return {
            "RoadCurvature": road_curvature,
            "RoadGrade": road_grade,
            "RoadFrictionCoefficient": friction,
            "VisibilityConditions": visibility,
            "target_town": target_town
        }

    def apply_env_config (self, env_config):
        resolved_env = self.resolve_env_config(env_config)

        road_curvature = resolved_env ["RoadCurvature"]
        road_grade = resolved_env ["RoadGrade"]
        road_condition = resolved_env ["RoadFrictionCoefficient"]
        visibility = resolved_env ["VisibilityConditions"]

        self.set_env_weather(
            road_condition=road_condition,
            visibility=visibility,
            friction_center_transform=self.ego_spawn_point
        )

        print(
            f"RoadCurvature: {road_curvature}, "
            f"RoadGrade: {road_grade}, "
            f"RoadFrictionCoefficient: {road_condition}, "
            f"VisibilityConditions: {visibility}"
        )

    def set_env_weather (self, road_condition="Dry", visibility="Good", friction_center_transform=None):
        weather = carla.WeatherParameters(
            cloudiness=10.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=5.0,
            fog_density=0.0,
            wetness=0.0,
            sun_azimuth_angle=0.0,
            sun_altitude_angle=45.0
        )

        if road_condition == "Slippery":
            weather.wetness = 80.0
            self.apply_road_friction(
                road_condition="Slippery",
                trigger_transform=friction_center_transform
            )
        elif road_condition == "Dry":
            if hasattr(self, "friction_trigger") and self.friction_trigger is not None:
                self.friction_trigger.destroy()
                self.friction_trigger = None
        else:
            raise ValueError(f"Invalid road_condition: {road_condition}")

        if visibility == "Good":
            pass
        elif visibility == "Nighttime":
            weather.sun_altitude_angle = -30.0
            weather.cloudiness = 40.0
        elif visibility == "AdverseWeather":
            weather.precipitation = 50.0
            weather.wetness = max(weather.wetness, 50.0)
            weather.cloudiness = 50.0
            weather.precipitation_deposits = 20.0
            weather.fog_density = 25.0
        else:
            raise ValueError(f"Invalid visibility: {visibility}")

        self.world.set_weather(weather)
        print(f"Weather set to road_condition={road_condition}, visibility={visibility}")

    def apply_road_friction (self, road_condition, trigger_transform=None, extent=None):
        if hasattr(self, "friction_trigger") and self.friction_trigger is not None:
            self.friction_trigger.destroy()
            self.friction_trigger = None

        if road_condition == "Dry":
            print("Road condition: Dry, keep default friction.")
            return

        if road_condition != "Slippery":
            raise ValueError(f"Invalid road condition: {road_condition}")

        bp = self.world.get_blueprint_library().find("static.trigger.friction")
        bp.set_attribute("friction", "0.3")

        if extent is None:
            extent = carla.Location(50000, 50000, 50000)

        bp.set_attribute("extent_x", str(extent.x))
        bp.set_attribute("extent_y", str(extent.y))
        bp.set_attribute("extent_z", str(extent.z))

        if trigger_transform is None:
            trigger_transform = carla.Transform(carla.Location(0, 0, 0))
            print("trigger_transform is None, fallback to (0, 0, 0)")

        self.friction_trigger = self.world.spawn_actor(bp, trigger_transform)
        print(f"Friction trigger spawned at {trigger_transform.location}")

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
        print(f"waypoints: {waypoints}")
        for i in range(len(waypoints) - 1):
            # 输出每一个点的坐标
            # print("`````````````````````````````````````````````````````````")

            debug.draw_line(waypoints [i] [0].transform.location + carla.Location(z=0.5),
                            waypoints [i + 1] [0].transform.location + carla.Location(z=0.5),
                            thickness=thickness, color=color, life_time=life_time)
            print(f"({waypoints [i] [0].transform.location.x}, {waypoints [i] [0].transform.location.y})")
            print("draw line")

    def draw_current_point (self, current_point):
        self.world.debug.draw_point(current_point, size=0.1, color=carla.Color(b=255), life_time=10)

    def set_spectator (self, vehicle):
        self.world.get_spectator().set_transform(
            carla.Transform(vehicle.get_transform().location +
                            carla.Location(z=30), carla.Rotation(pitch=-90))
        )

    # def set_spectator (self, vehicle):
    #     spectator = self.world.get_spectator()
    #     transform = vehicle.get_transform()
    #
    #     location = transform.location
    #     forward = transform.get_forward_vector()
    #     right = transform.get_right_vector()
    #     up = transform.get_up_vector()
    #
    #     camera_loc = location + forward * 1.0 + right * -0.35 + up * 1.15
    #     camera_rot = transform.rotation
    #
    #     spectator.set_transform(carla.Transform(camera_loc, camera_rot))

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

    def get_ego_spawn_config (self, road_curvature="Straight", road_grade="Flat", lane_id=5):


        # 这里只是示例值，你后面换成你实际标定好的 x
        if road_grade == "Flat":
            base_y = 237.568939
            y_position = base_y + (5 - lane_id) * 3.5
            if road_curvature == "Curve":
                start_x = 590
            else:
                start_x = 100
        elif road_grade == "Uphill":
            base_y = 37
            y_position = base_y - (lane_id-1) * 3.5
            start_x = -355  # 上坡起点
        elif road_grade == "Downhill":
            base_y = 38.5
            y_position = base_y - (lane_id - 1) * 3.5
            start_x = 110  # 下坡起点
        else:
            raise ValueError(f"Unsupported road_grade: {road_grade}")

        temp_location = carla.Location(x=start_x, y=y_position, z=0.0)

        # 从地图获取该点附近道路的 waypoint
        waypoint = self.world.get_map().get_waypoint(
            temp_location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )

        if waypoint is None:
            raise RuntimeError(f"Cannot find waypoint for x={start_x}, y={y_position}")

        road_z = waypoint.transform.location.z

        # 在路面高度基础上加一点，防止刷进地里
        return carla.Location(x=start_x, y=y_position, z=road_z + 1.0)

    def init_ego (self, testcase, env_config=None, agent_type='behavior', behavior='cautious'):
        ego_statement = None

        for statement in testcase.constructor_statements:
            if statement.assignee == 'ego':
                ego_statement = statement
                break

        if ego_statement is None:
            raise ValueError("Ego statement not found in testcase.")

        lane_id = ego_statement.args.get('lane_id', 5)

        resolved_env = self.resolve_env_config(env_config)
        road_curvature = resolved_env ["RoadCurvature"]
        road_grade = resolved_env ["RoadGrade"]

        start_location = self.get_ego_spawn_config(
            road_curvature=road_curvature,
            road_grade=road_grade,
            lane_id=lane_id
        )

        start_point = carla.Transform(start_location, carla.Rotation(yaw=0))
        start_waypoint = self.world.get_map().get_waypoint(start_location)
        forward_vec = start_point.rotation.get_forward_vector()

        self.ego_vehicle = self.spawn_ego_by_point(start_point)
        self.ego_spawn_point = start_point
        self.ego_location = start_location
        self.world.tick()

        if self.ego_vehicle is not None:
            print(f"Ego vehicle spawned at {start_location}")
        else:
            print("Ego vehicle spawn failed.")
            return None

        initial_speed = ego_statement.args.get('initial_speed', 20)
        # if initial_speed > 0:
        #     initial_speed_ms = initial_speed * (1000 / 3600)
        #     self.ego_vehicle.set_target_velocity(
        #         carla.Vector3D(
        #             x=forward_vec.x * initial_speed_ms,
        #             y=forward_vec.y * initial_speed_ms,
        #             z=forward_vec.z * initial_speed_ms
        #         )

        #
        if initial_speed > 0:
            initial_speed_ms = initial_speed * (1000 / 3600)
            self.ego_vehicle.set_target_velocity(
                carla.Vector3D(
                    x=0,
                    y=0,
                    z=0
                )
            )
            print(f"Ego vehicle initial speed set to {initial_speed} km/h")
        for _ in range(3):
            self.world.tick()
        next_waypoint = start_waypoint.next(200) [0]

        if agent_type == 'basic':
            self.agent = BasicAgent(self.ego_vehicle)
            self.agent.set_target_speed(initial_speed)
            self.agent.set_destination(next_waypoint.transform.location)
            print(f"Ego vehicle destination set to {next_waypoint.transform.location}")
        elif agent_type == 'behavior':
            self.agent = BehaviorAgent(self.ego_vehicle, behavior=behavior)
            self.agent.set_destination(start_location, next_waypoint.transform.location)
            print(f"Ego vehicle destination set to {next_waypoint.transform.location}")
        else:
            raise ValueError(f"Unknown agent type: {agent_type}")

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
                
                # 验证 lane_id 的有效性
                if npc_lane_id < 1:
                    npc_lane_id = 1
                    statement.args['lane_id'] = 1
                

                # MAX_LANE_ID = X  # 在这里设置最大值
                # if npc_lane_id > MAX_LANE_ID:
                #     logger.warning(f"Invalid lane_id {npc_lane_id} for NPC '{statement.assignee}', setting to maximum value {MAX_LANE_ID}")
                #     npc_lane_id = MAX_LANE_ID
                #     statement.args['lane_id'] = MAX_LANE_ID
                
                print(f"NPC vehicle lane_id: {npc_lane_id}, offset: {npc_offset}, initial_speed: {npc_initial_speed}")
                # 计算 NPC 车辆的偏移量
                id_offset, dis_offset = calcu_offset(npc_lane_id, ego_lane_id, npc_offset, ego_offset)
                # 生成 NPC 车辆
                npc_transform = self.spawn_npc_by_lane_offset(self.ego_spawn_point, id_offset, dis_offset)
                self.world.tick()
                if npc_transform:
                    npc_vehicle = self.spawn_npc_by_point(npc_transform)
                    print(f"NPC vehicle {npc_vehicle.id} spawned at {npc_transform.location}")

                    npc_initial_speed_ms = npc_initial_speed * (1000 / 3600)

                    forward_vector = npc_transform.get_forward_vector()
                    init_velocity = carla.Vector3D(
                        x=forward_vector.x * npc_initial_speed_ms,
                        y=forward_vector.y * npc_initial_speed_ms,
                        z=forward_vector.z * npc_initial_speed_ms
                    )

                    npc_vehicle.set_target_velocity(init_velocity)
                    print(f"NPC vehicle {npc_vehicle.id} initial speed set to {npc_initial_speed} km/h")

                    self.lane_change_in_progress [npc_vehicle.id] = False
                    self.lane_follow_in_progress [npc_vehicle.id] = False

                    self.npc_target_speeds [npc_vehicle.id] = npc_initial_speed
                    self.npc_follow_lane_generators [npc_vehicle.id] = self.npc_follow_lane(
                        npc_vehicle,
                        npc_initial_speed
                    )
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

    def npc_follow_lane (self, npc_vehicle, target_speed):
        args_lateral_dict = {'K_P': 0.5, 'K_D': 0.2, 'K_I': 0.0, 'dt': 1.0 / 20.0}
        args_long_dict = {'K_P': 0.3, 'K_D': 0.0, 'K_I': 0.05, 'dt': 1.0 / 20.0}

        pid_controller = VehiclePIDController(
            npc_vehicle,
            args_lateral=args_lateral_dict,
            args_longitudinal=args_long_dict
        )

        while True:
            if npc_vehicle is None:
                break

            current_location = npc_vehicle.get_location()
            current_waypoint = self.map.get_waypoint(
                current_location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )

            if current_waypoint is None:
                print(f"[FOLLOW_LANE] NPC {npc_vehicle.id}: current waypoint not found")
                break

            next_waypoints = current_waypoint.next(2.0)
            if len(next_waypoints) == 0:
                print(f"[FOLLOW_LANE] NPC {npc_vehicle.id}: next waypoint not found")
                break

            target_waypoint = next_waypoints [0]
            control = pid_controller.run_step(target_speed, target_waypoint)
            npc_vehicle.apply_control(control)

            yield

    def active_npc_action (self, npc_vehicle, action_name, args, testcase):
        print("-----------------------------------")
        args_lateral_dict = {'K_P': 0.9, 'K_D': 1.2, 'K_I': 0.2, 'dt': 1.0 / 20.0}
        args_long_dict = {'K_P': 0.5, 'K_D': 0.0, 'K_I': 0.75, 'dt': 1.0 / 20.0}
        pid_controller = VehiclePIDController(npc_vehicle, args_lateral_dict, args_long_dict)

        if action_name in ["accelerate", "decelerate"]:
            target_speed = args ["target_speed"]
            print(f"NPC vehicle {npc_vehicle.id} {action_name} to {target_speed} km/h")

            # 可调参数
            speed_tolerance = args.get("speed_tolerance", 0.3)  # 到目标速度的容忍误差
            max_control_ticks = args.get("max_control_ticks", 100)  # 最多控制多久
            min_control_ticks = args.get("min_control_ticks", 10)  # 至少控制多久，避免1 tick就结束
            hold_ticks = args.get("hold_ticks", 20)  # 达到目标后继续沿车道保持的时间
            waypoint_step = args.get("waypoint_step", 2.0)  # 前视距离
            brake_cap = args.get("brake_cap", 0.1)  # 例如 0.2；None 表示不限制

            # -------------------------------
            # 特殊情况：目标速度为 0
            # -------------------------------
            if target_speed == 0.0:
                max_stop_ticks = args.get("max_stop_ticks", 100)

                for tick_idx in range(max_stop_ticks):
                    speed_before = get_speed(npc_vehicle)

                    stop_control = carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0 if brake_cap is None else min(1.0, brake_cap),
                        steer=0.0
                    )
                    npc_vehicle.apply_control(stop_control)

                    print(
                        f"[NPC {npc_vehicle.id}] STOP tick={tick_idx}, "
                        f"speed_before={speed_before:.2f} km/h, "
                        f"apply={stop_control}, actor={npc_vehicle.get_control()}"
                    )

                    yield

                    speed_after = get_speed(npc_vehicle)
                    print(f"[NPC {npc_vehicle.id}] STOP speed_after={speed_after:.2f} km/h")

                    if speed_after < 0.5:
                        break

                print(f"NPC vehicle {npc_vehicle.id} stopped")
                return

            # -------------------------------
            # 非零目标速度：持续控制直到接近目标速度
            # -------------------------------
            tick_counter = 0

            for tick_idx in range(max_control_ticks):
                current_location = npc_vehicle.get_location()
                current_waypoint = self.map.get_waypoint(
                    current_location,
                    project_to_road=True,
                    lane_type=carla.LaneType.Driving
                )

                if current_waypoint is None:
                    print(f"NPC vehicle {npc_vehicle.id}: current waypoint not found")
                    break

                next_waypoints = current_waypoint.next(waypoint_step)
                if len(next_waypoints) == 0:
                    print(f"NPC vehicle {npc_vehicle.id}: next waypoint not found")
                    break

                target_waypoint = next_waypoints [0]

                speed_before = get_speed(npc_vehicle)
                control = pid_controller.run_step(target_speed, target_waypoint)

                # 真正对 brake 做限幅（如果你想测 brake 限制，就传 brake_cap）
                if brake_cap is not None:
                    control.brake = min(control.brake, brake_cap)

                npc_vehicle.apply_control(control)

                print(
                    f"[NPC {npc_vehicle.id}] tick={tick_idx}, "
                    f"speed_before={speed_before:.2f} km/h, "
                    f"target={target_speed:.2f} km/h, "
                    f"apply={control}, actor={npc_vehicle.get_control()}"
                )

                yield

                speed_after = get_speed(npc_vehicle)
                tick_counter += 1

                print(
                    f"[NPC {npc_vehicle.id}] tick={tick_idx}, "
                    f"speed_after={speed_after:.2f} km/h, "
                    f"speed_error={abs(speed_after - target_speed):.2f}"
                )

                # 注意：这里用的是 yield 之后的 speed_after，不是旧速度
                if tick_counter >= min_control_ticks and abs(speed_after - target_speed) <= speed_tolerance:
                    break
            self.npc_target_speeds [npc_vehicle.id] = target_speed
            print(f"NPC vehicle {npc_vehicle.id} target speed set to {target_speed} km/h")

            # -------------------------------
            # 达到目标后，不立刻失控
            # 继续沿当前车道保持一小段时间
            # -------------------------------
            for hold_idx in range(hold_ticks):
                current_location = npc_vehicle.get_location()
                current_waypoint = self.map.get_waypoint(
                    current_location,
                    project_to_road=True,
                    lane_type=carla.LaneType.Driving
                )

                if current_waypoint is None:
                    print(f"NPC vehicle {npc_vehicle.id}: hold current waypoint not found")
                    break

                next_waypoints = current_waypoint.next(waypoint_step)
                if len(next_waypoints) == 0:
                    print(f"NPC vehicle {npc_vehicle.id}: hold next waypoint not found")
                    break

                target_waypoint = next_waypoints [0]
                hold_speed = get_speed(npc_vehicle)

                hold_control = pid_controller.run_step(hold_speed, target_waypoint)

                if brake_cap is not None:
                    hold_control.brake = min(hold_control.brake, brake_cap)

                npc_vehicle.apply_control(hold_control)

                print(
                    f"[NPC {npc_vehicle.id}] HOLD tick={hold_idx}, "
                    f"hold_speed={hold_speed:.2f} km/h, "
                    f"apply={hold_control}, actor={npc_vehicle.get_control()}"
                )

                yield

            return

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

            next_lane_change_interval = get_next_lane_change_interval(
                testcase,
                npc_assignee,
                args ["trigger_sequence"]
            )

            yield from self.execute_lane_changes(npc_vehicle, current_lane_id, target_lane_id, target_speed,
                                                 pid_controller, time_limit=next_lane_change_interval)

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

    def execute_lane_changes (self, npc_vehicle, current_lane_id, target_lane_id, target_speed, pid_controller=None,
                              time_limit=5):
        #
        print(f"Current lane ID: {current_lane_id}, Target lane ID: {target_lane_id}")
        # 计算需要跨越的车道数量
        lane_num = abs(target_lane_id - current_lane_id)
        print(f"Lane change distance: {lane_num} lanes")
        for _ in range(lane_num):
            direction = "left" if current_lane_id < target_lane_id else "right"
            yield from self.lane_change_direction(npc_vehicle, direction, target_speed=target_speed,
                                                  pid_controller=pid_controller, time_limit=time_limit)
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

    def cal_target_route (self, vehicle=None, lanechange="left", target_dis=0):
        # grp = NpcGlobalRoutePlanner(self.map, 2)
        grp = GlobalRoutePlanner(self.map, 2)
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


    def lane_change_direction (self, npc_vehicle, direction, target_speed, pid_controller=None, time_limit=5):
        """异步执行换道操作"""
        vehicle_id = npc_vehicle.id

        # 检查是否已经在换道
        if self.lane_change_in_progress.get(vehicle_id, False):
            print(f"Lane change already in progress for NPC {vehicle_id}. Skipping action.")
            return

        # 标记为正在换道
        self.lane_change_in_progress [vehicle_id] = True
        print(f"Starting lane change for NPC {npc_vehicle.id}...")

        # 计算目标路径
        route = self.cal_target_route(vehicle=npc_vehicle, lanechange=direction, target_dis=max(target_speed * 1.3, 20))
        self.draw_target_line(route)
        print("draw----------yellow")

        # 初始化换道参数
        waypoint_index = 0
        arrive_target_point = False
        target_distance_threshold = (target_speed * 1 / 3.6) * 0.5  # 转换为km/h
        start_time = self.world.get_snapshot().timestamp.elapsed_seconds
        print(f"Start time: {start_time}")

        try:
            npc_speed = None
            while not arrive_target_point:
                current_time = self.world.get_snapshot().timestamp.elapsed_seconds
                print(f"Current time --: {current_time}")
                print(f"相差时间: {current_time - start_time}")

                # 检查时间限制
                if current_time - start_time > time_limit:
                    print(f"Time limit exceeded for NPC {npc_vehicle.id}. Stopping action.")
                    self.lane_change_in_progress [vehicle_id] = False
                    raise ActionException("Time limit exceeded for NPC lane change.")

                # 检查是否到达目标点
                if waypoint_index < len(route):
                    target_waypoint = route [waypoint_index] [0]
                    transform = npc_vehicle.get_transform()
                    self.world.debug.draw_point(transform.location, size=0.1, color=carla.Color(b=255), life_time=25)
                    print(f"draw-------------------------")

                    distance_to_waypoint = distance_vehicle(target_waypoint, transform)
                    if distance_to_waypoint < target_distance_threshold:
                        waypoint_index += 1
                        if waypoint_index >= len(route):
                            arrive_target_point = True
                            print(f"NPC {npc_vehicle.id} has arrived at the target point.")
                            break
                    else:
                        control = pid_controller.run_step(target_speed, target_waypoint)
                        print(f"NPC lane change {npc_vehicle.id} control: {control}")
                        npc_vehicle.apply_control(control)
                        print(f"npc control: {control}")
                        npc_speed = get_speed(npc_vehicle)
                        angular_velocity = npc_vehicle.get_angular_velocity()
                        yaw_rate = angular_velocity.z
                        print(f"NPC {npc_vehicle.id} speed: {npc_speed} km/h")
                else:
                    break

                yield

        finally:
            # 一旦换道完成
            npc_location = npc_vehicle.get_transform().location
            print(f"NPC {npc_vehicle.id} has stopped at {npc_location}.")
            npc_waypoint = carla.Transform(npc_location, carla.Rotation(yaw=0))
            npc_forward_vec = npc_waypoint.rotation.get_forward_vector()


            self.npc_target_speeds [npc_vehicle.id] = npc_speed if npc_speed is not None else target_speed
            npc_vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=0))
            self.lane_change_in_progress [vehicle_id] = False
            print(f"NPC {npc_vehicle.id} has completed the lane change.")
