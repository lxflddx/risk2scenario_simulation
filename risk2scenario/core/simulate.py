import logging
import time
import traceback
from risk2scenario.utils import simulate_utlis, DummyWorld
# print(sys.path)
# print(sys.executable)
import carla

from .carla_world import CarlaWorld
# from carla_world import ActionException

# 修改成自己的carla路径
# sys.path.append(
#     'F://CARLA_0.9.15//WindowsNoEditor//PythonAPI//carla')  # 如果是basicagent的话需要用这个路径，如果是behavioragent的话直接用目前这个代码里面的就可以
from agents.tools.misc import distance_vehicle, get_speed
from risk2scenario.utils.my_parse import Parser

logger = logging.getLogger(__name__)


def safe_destroy_actor (actor):
    try:
        if actor is not None:
            actor.destroy()
            time.sleep(0.1)
            print(f"Successfully destroyed actor {actor.id}")
    except Exception as e:
        print(f"WARNING: Failed to destroy actor {getattr(actor, 'id', 'Unknown')} due to {e}")
        traceback.print_exc()


# class Simulate:
class Simulation:
    # def __init__(self):
    #     self.CARLA = CarlaWorld()

    def __init__ (self, max_retries=3):
        self.CARLA = None
        self.collision_sensor = None
        self.is_collision = False
        self.evaluation_result = None
        self.collision_vehicle_id = None
        self._collision_counter = 0
        for attempt in range(max_retries):
            try:
                self.CARLA = CarlaWorld()
                break
            except ConnectionError as e:
                if attempt < max_retries - 1:
                    print(f"仿真初始化失败，30秒后重试... ({attempt + 1}/{max_retries})")
                    time.sleep(30)
                else:
                    raise RuntimeError(f"仿真初始化最终失败: {str(e)}")

    def apply_env_config (self, env_config):
        weather = self.CARLA.world.get_weather()

    def analyze_collision (self, event):
        """
        分析碰撞类型
        返回: True=追尾碰撞(需排除) / False=有效碰撞(需记录)
        """
        ego_vehicle = event.actor
        other_vehicle = event.other_actor

        # 获取两车的位置和方向信息
        ego_transform = ego_vehicle.get_transform()
        print(f"ego_transform: {ego_transform}")
        other_transform = other_vehicle.get_transform()
        print(f"other_transform: {other_transform}")

        # 计算相对位置向量（从NPC指向ego车）
        delta = ego_transform.location - other_transform.location
        delta_2d = carla.Vector3D(delta.x, delta.y, 0).make_unit_vector()
        print(f"delta_2d: {delta_2d}")

        # 获取两车的前向向量（2D平面）
        ego_forward = ego_transform.get_forward_vector()
        ego_forward_2d = carla.Vector3D(ego_forward.x, ego_forward.y, 0).make_unit_vector()
        print(f"ego_forward:{ego_forward_2d}")
        other_forward = other_transform.get_forward_vector()
        other_forward_2d = carla.Vector3D(other_forward.x, other_forward.y, 0).make_unit_vector()
        print(f"other_forward:{other_forward_2d}")

        # 判断NPC车辆是否在主车的后方
        is_behind = delta.dot(ego_forward_2d) > 0  # NPC车辆在主车的后方

        # 判断追尾碰撞的条件
        is_rear_end = (
                is_behind and
                other_forward_2d.dot(delta_2d) > 0.7 and  # NPC车头指向主车
                other_vehicle.get_velocity().dot(delta_2d) > 0  # NPC向主车移动
        )
        return is_rear_end

    def run_test (self, testcase, env_config=None, agent_type='basic'):
        global trigger_sequence
        print()
        time.sleep(1)
        settings = self.CARLA.world.get_settings()
        print(f"CARLA settings: {settings}")
        self.CARLA.world.tick()
        if settings.synchronous_mode and settings.fixed_delta_seconds == 0.05:
            print("Settings applied correctly.")
        else:
            print("Settings not applied correctly. Reapplying...")
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            self.CARLA.world.apply_settings(settings)
        if env_config:
            self.apply_env_config(env_config)
        target_town = 'Town06'
        self.CARLA.load_map_needed(target_town)
        collision_sensor = None

        def _on_collision (event):
            if not self.is_collision and self._collision_counter == 0:
                crash_vehicle = event.other_actor
                print(f"Collision detected with vehicle: {crash_vehicle.id}")
                # if crash_vehicle.id == 0:
                #     print("碰撞对象为 ID 0（已忽略）")
                #     logger.info(f"Collision with vehicle ID 0 ignored")
                #     return
                # 分析碰撞类型
                is_rear_end = self.analyze_collision(event)

                if is_rear_end:
                    print("追尾碰撞（已忽略）")
                    logger.info(f"Rear-end collision with vehicle {crash_vehicle.id} ignored")
                    self._collision_counter += 1
                else:
                    print("---------------------------- FIRST VALID COLLISION ----------------------------")
                    print(f"首次有效碰撞（已记录）车辆ID: {crash_vehicle.id}")
                    self.is_collision = True
                    self._collision_counter += 1
                    self.collision_vehicle_id = crash_vehicle.id

        try:
            self.CARLA.init_ego(testcase, agent_type=agent_type)
            self.CARLA.init_npc(testcase)
            collision_bp = self.CARLA.world.get_blueprint_library().find('sensor.other.collision')
            collision_sensor = self.CARLA.world.spawn_actor(collision_bp, carla.Transform(),
                                                            attach_to=self.CARLA.ego_vehicle)
            collision_sensor.listen(_on_collision)

            min_distance = float('inf')
            ego_last_speed = get_speed(self.CARLA.ego_vehicle)
            # ego_last_speed = CARLA.ego_vehicle.get_velocity().length()
            print(f"Ego vehicle speed: {ego_last_speed}")
            ego_acc_list = []
            ego_speed_list = []
            ego_velocity_list = []
            npc_velocity_list = []
            ttc_list = []
            relative_speed_list = []
            distance_list = []
            time_interval = 0.05
            tick_count = 0
            recording_started = False
            speed_threshold = 2.5
            print("Starting the simulation...")
            action_queue = []
            for statement in testcase.constructor_statements:
                if statement.assignee == 'ego':
                    ego_statement = statement
                    break
            ego_initial_speed = ego_statement.args.get('initial_speed', 20)

            trigger_times = []

            # 在仿真开始之前计算所有动作的触发时间
            # 首先，根据 trigger_sequence 对动作进行排序
            sorted_statements = sorted(testcase.method_statements, key=lambda x: x.args ["trigger_sequence"])

            # 然后，计算每个动作的触发时间
            trigger_time_map = {}  # 用于存储每个 trigger_sequence 的触发时间
            for statement in sorted_statements:
                npc_index = statement.callee
                time_interval = statement.args.get("time_interval", 5)  # 获取时间间隔，默认值为5秒
                trigger_sequence = statement.args ["trigger_sequence"]

                if trigger_sequence == 1:
                    # 如果是第一个动作，触发时间为时间间隔
                    trigger_time = time_interval * 20
                else:
                    # 否则，触发时间为前一个 trigger_sequence 的触发时间加上时间间隔
                    previous_trigger_sequence = trigger_sequence - 1
                    previous_trigger_time = trigger_time_map.get(previous_trigger_sequence, 0)
                    trigger_time = previous_trigger_time + time_interval * 20

                trigger_times.append((trigger_time, statement))
                trigger_time_map [trigger_sequence] = trigger_time  # 更新 trigger_sequence 的触发时间
            triggered_actions = set()
            while tick_count < 600:
                time.sleep(0.01)
                print(f"current time: {tick_count / 20:.2f}")
                self.CARLA.world.tick()
                if agent_type == 'basic':
                    control = self.CARLA.agent.run_step()
                    print("basic agent")
                if agent_type == 'behavior':
                    dummy_world = DummyWorld.DummyWorld(self.CARLA.world, self.CARLA.ego_vehicle)
                    self.CARLA.agent.update_information(dummy_world)
                    if len(self.CARLA.agent._local_planner.waypoints_queue) < 1:
                        print('======== Success, Arrivied at Target Point!')
                        break
                    speed_limit = self.CARLA.ego_vehicle.get_speed_limit()
                    self.CARLA.agent.get_local_planner().set_speed(speed_limit)
                    control = self.CARLA.agent.run_step(debug=True)
                print(f"ego Control: {control}")
                self.CARLA.set_spectator(self.CARLA.ego_vehicle)
                self.CARLA.ego_vehicle.apply_control(control)
                ego_current_speed = get_speed(self.CARLA.ego_vehicle)
                if not recording_started:
                    if abs(ego_current_speed - ego_initial_speed) <= speed_threshold:
                        print(f"Speed is close to target speed ({ego_initial_speed} km/h). Starting to record.")
                        recording_started = True
                if recording_started:
                    ego_speed_list.append(ego_current_speed)
                    print(f"Ego vehicle speed: {ego_current_speed:.2f} km/h")
                ego_crrent_location = self.CARLA.ego_vehicle.get_location()
                print(f"Ego vehicle location: {ego_crrent_location}")

                # Simulate for 30 seconds
                # 调度器，在每个tick检测是否有需要执行的动作，如果需要，将动作添加到队列中，然后在下一个tick将队列里的动作执行，并清空队列
                # 判断：时间是否到达触发条件/动作还没有执行完毕
                # 检查是否有需要执行的动作
                for trigger_time, statement in trigger_times:
                    if tick_count == trigger_time:
                        npc_index = statement.callee
                        action_name = statement.method_name
                        npc_info = self.CARLA.npc_list [npc_index]
                        npc_vehicle_id = npc_info ["vehicle_id"]
                        npc_vehicle = self.CARLA.world.get_actor(npc_vehicle_id)
                        print(
                            f"Trigger Action: {action_name} for NPC vehicle {npc_vehicle_id} at time {tick_count / 20:.2f}")
                        gen = self.CARLA.active_npc_action(npc_vehicle, action_name, statement.args, testcase)
                        action_queue.append(gen)
                        # triggered_actions.add(trigger_sequence)

                # 执行动作队列中的动作
                # 在tick循环中
                completed_indices = []
                for i, action_gen in enumerate(action_queue):
                    try:
                        next(action_gen)
                    except StopIteration:
                        completed_indices.append(i)
                    except ActionException as e:
                        print(f"Action terminated: {str(e)}")
                        completed_indices.append(i)

                        npc_vehicle.apply_control(carla.VehicleControl())

                # 反向移除完成项
                for i in sorted(completed_indices, reverse=True):
                    removed = action_queue.pop(i)
                    print(f"Removed action {i} from queue")

                tick_count += 1

                # 计算最小距离和TTC
                ego_transform = self.CARLA.ego_vehicle.get_transform()
                for npc_info in self.CARLA.npc_list.values():
                    npc_vehicle = self.CARLA.world.get_actor(npc_info ["vehicle_id"])
                    if npc_vehicle:
                        # 计算TTC
                        # ttc, distance, relative_speed, ego_velocity, target_velocity = simulate_utlis.calculate_ttc(CARLA.ego_vehicle,
                        #                                                                              npc_vehicle)
                        # relative_speed_list.append(relative_speed)
                        # distance_list.append(distance)
                        # ttc_list.append(ttc)
                        # ego_velocity_list.append(ego_velocity.x)
                        # npc_velocity_list.append(target_velocity.x)
                        npc_transform = npc_vehicle.get_transform()
                        npc_speed = npc_vehicle.get_velocity()
                        print(f"NPC vehicle  speed{npc_speed.x, npc_speed.y}")
                        npc_waypoint = self.CARLA.map.get_waypoint(npc_transform.location)
                        distance = distance_vehicle(npc_waypoint, ego_transform)
                        if distance < min_distance:
                            min_distance = distance
                            print(f"Min distance: {min_distance:.2f} m")
                # time.sleep(0.05)

            # 计算主车加速度变化率
            ego_acc_list = []
            for i in range(1, len(ego_speed_list)):
                ego_acc = (ego_speed_list [i] - ego_speed_list [i - 1]) / time_interval
                ego_acc_list.append(ego_acc)
            acr = simulate_utlis.acc_check(ego_acc_list)
            evaluation_result = [min_distance, -acr]
            print(f"Evaluation result: {evaluation_result}")

        except Exception as e:
            print(f"An error occurred: {e}")
            evaluation_result = [float('inf'), 0]
            traceback.print_exc()


        finally:
            print("Cleaning up the simulation...")
            safe_destroy_actor(collision_sensor)
            self.CARLA.world.tick()
            safe_destroy_actor(getattr(self.CARLA, 'ego_vehicle', None))
            self.CARLA.world.tick()
            for npc_info in self.CARLA.npc_list.values():
                safe_destroy_actor(self.CARLA.world.get_actor(npc_info ["vehicle_id"]))
                self.CARLA.world.tick()
            safe_destroy_actor(getattr(self.CARLA, 'friction_trigger', None))
            settings = self.CARLA.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self.CARLA.world.apply_settings(settings)
            self.CARLA.world.tick()
            time.sleep(0.5)

            # if ego_speed_list:
            #     print(f"Ego speed list: {ego_speed_list}")
            #     simulate_utlis.plot_speed_graph(ego_speed_list)

            # if ego_acc_list:
            #     simulate_utlis.plot_acceleration_graph(ego_acc_list)
            # if ttc_list:
            #     print(f"TTC list: {ttc_list}")
            #     simulate_utlis.plot_ttc_graph(ttc_list)
            # if relative_speed_list:
            #     print(f"Relative speed list: {relative_speed_list}")
            # if distance_list:
            #     print(f"Distance list: {distance_list}")
            # if ego_velocity_list:
            #     print(f"Ego velocity list: {ego_velocity_list}")
            # if npc_velocity_list:
            #     print(f"NPC velocity list: {npc_velocity_list}")
        return evaluation_result, self.is_collision


if __name__ == '__main__':
    parser = Parser()
    testcase_str = """
def testcase(self):
    vehicle1 = NPC(lane_id=2, offset=45.0, initial_speed=21.51951588050754)
    ego = NPC(lane_id=2, offset=30.0, initial_speed=6.1961523814927455)
    vehicle3 = NPC(lane_id=3, offset=20.759637511685536, initial_speed=29.238527202960007)
    vehicle1.accelerate(target_speed=57.90023887828039, trigger_sequence=1, time_interval=5)
    vehicle1.changeLane(target_lane=3, target_speed=18.889083232878914, trigger_sequence=2, time_interval=4)
    vehicle1.decelerate(target_speed=0.3236302326904904, trigger_sequence=3, time_interval=5)
    vehicle3.decelerate(target_speed=0.08105255515259768, trigger_sequence=1, time_interval=5)

        """

    test_case = parser.parse_testcase_string(testcase_str)

    # for i in range(1, 3):
    #     time.sleep(1)
    #     try:
    #         simulation = Simulation()
    #         # ==== 新增连接检查 ====
    #         if not simulation.CARLA.check_connection():
    #             print("检测到连接断开，尝试恢复...")
    #             if not simulation.CARLA.soft_reset():
    #                 print("恢复失败，重新创建CarlaWorld实例")
    #                 simulation.CARLA = CarlaWorld()  # 强制重建
    #
    #         result, is_collision = simulation.run_test(test_case)
    #         print(f"Result: {result}, Collision: {is_collision}")
    #     except RuntimeError as e:
    #         print(f"捕获到运行时错误: {str(e)}")
    #         # ==== 新增应急处理 ====
    #         if "time-out" in str(e):
    #             print("尝试紧急恢复连接...")
    #             try:
    #                 simulation.CARLA.client = carla.Client('localhost', 2000)
    #                 simulation.CARLA.client.set_timeout(10.0)
    #                 simulation.CARLA.world = simulation.CARLA.client.get_world()
    #                 print("紧急恢复成功")
    #             except:
    #                 print("紧急恢复失败，建议重启服务器")
    #     finally:
    #         # 可选：添加轻量级清理
    #         if hasattr(simulation, 'CARLA') and simulation.CARLA.ego_vehicle:
    #             simulation.CARLA.ego_vehicle.destroy()
    # for i in range(1, 2):
    #     time.sleep(1)
    simulation = Simulation()
    result, is_collision = simulation.run_test(test_case)
    print(f"Result: {result}, Collision: {is_collision}")

    # vehicle4 = NPC(lane_id=1, offset=20, initial_speed=20)
    # vehicle2.changeLane(target_lane=2, target_speed=15, trigger_sequence=1)
    # vehicle1.changeLane(target_lane=2, target_speed=15, trigger_sequence=2)
