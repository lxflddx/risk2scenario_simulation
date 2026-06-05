import logging
import yaml
from datetime import datetime
from risk2scenario.utils.my_parse import Parser
from risk2scenario.core.simulate import Simulation


import random

class NPC:
    def __init__(self, lane_id: int, offset: float, initial_speed: float):
        self.lane_id = lane_id
        self.offset = offset
        self.initial_speed = initial_speed

    def accelerate(self, target_speed: float, trigger_sequence: int):
        pass

    def decelerate(self, target_speed: float, trigger_sequence: int):
        pass

    def changeLane(self, target_lane: int, target_speed: float, trigger_sequence: int):
        pass


def generate_testcase_without_ego():
    num_vehicles = random.randint(2, 4)  # 随机生成车辆数量
    vehicles = []
    vehicle_names = []
    testcase_str = "def testcase(self):\n"

    # 随机生成车辆的初始状态
    for i in range(num_vehicles):
        lane_id = random.choice([1, 2, 3, 4, 5])
        offset = round(random.uniform(0, 50), 8)
        initial_speed = round(random.uniform(0, 40), 8)
        vehicle = NPC(lane_id, offset, initial_speed)
        vehicles.append(vehicle)
        vehicle_names.append(f"vehicle{i + 1}")
        testcase_str += f"    vehicle{i + 1} = NPC(lane_id={lane_id}, offset={offset}, initial_speed={initial_speed})\n"

    # 为NPC车辆随机生成动作
    actions = []  # 用于存储所有动作
    current_sequence = 1  # 初始化全局时间序列
    while current_sequence <= 5:  # 确保时间戳不超过5
        for i, vehicle in enumerate(vehicles):
            num_actions = random.randint(0, 1)  # 每个时间戳随机生成0或1个动作
            if num_actions > 0:
                action_type = random.choice(["accelerate", "decelerate", "changeLane"])
                if action_type == "accelerate":
                    target_speed = round(random.uniform(30, 60), 8)
                    actions.append((vehicle_names[i], "accelerate", target_speed, current_sequence))
                elif action_type == "decelerate":
                    target_speed = round(random.uniform(0, 30), 8)
                    actions.append((vehicle_names[i], "decelerate", target_speed, current_sequence))
                elif action_type == "changeLane":
                    target_lane = random.choice([1, 3])
                    target_speed = round(random.uniform(0, 60), 8)
                    actions.append((vehicle_names[i], "changeLane", target_lane, target_speed, current_sequence))
        current_sequence += 1  # 增加时间戳

    # 按时间戳排序并生成最终的测试用例字符串
    actions.sort(key=lambda x: x[-1])  # 按时间戳排序
    for action in actions:
        vehicle_name = action[0]
        action_type = action[1]
        if action_type == "accelerate":
            target_speed = action[2]
            trigger_sequence = action[3]
            testcase_str += f"    {vehicle_name}.accelerate(trigger_sequence={trigger_sequence}, target_speed={target_speed})\n"
        elif action_type == "decelerate":
            target_speed = action[2]
            trigger_sequence = action[3]
            testcase_str += f"    {vehicle_name}.decelerate(trigger_sequence={trigger_sequence}, target_speed={target_speed})\n"
        elif action_type == "changeLane":
            target_lane = action[2]
            target_speed = action[3]
            trigger_sequence = action[4]
            testcase_str += f"    {vehicle_name}.changeLane(trigger_sequence={trigger_sequence}, target_lane={target_lane}, target_speed={target_speed})\n"

    return testcase_str, vehicle_names, actions, vehicles


def generate_testcase():
    # 生成不带主车的测试用例
    testcase_str, vehicle_names, actions, vehicles = generate_testcase_without_ego()

    # 随机选择一个车辆作为主车
    ego_vehicle_index = random.randint(0, len(vehicle_names) - 1)
    ego_vehicle_name = vehicle_names[ego_vehicle_index]

    # 将主车的动作删除
    filtered_actions = [action for action in actions if action[0] != ego_vehicle_name]

    # 重新生成最终的测试用例字符串
    final_testcase_str = "def testcase(self):\n"
    for i, vehicle_name in enumerate(vehicle_names):
        if vehicle_name == ego_vehicle_name:
            final_testcase_str += f"    ego = NPC(lane_id={vehicles[i].lane_id}, offset={vehicles[i].offset}, initial_speed={vehicles[i].initial_speed})\n"
        else:
            final_testcase_str += f"    {vehicle_name} = NPC(lane_id={vehicles[i].lane_id}, offset={vehicles[i].offset}, initial_speed={vehicles[i].initial_speed})\n"

    # 添加动作
    for action in filtered_actions:
        vehicle_name = action[0]
        action_type = action[1]
        if action_type == "accelerate":
            target_speed = action[2]
            trigger_sequence = action[3]
            final_testcase_str += f"    {vehicle_name}.accelerate(trigger_sequence={trigger_sequence}, target_speed={target_speed})\n"
        elif action_type == "decelerate":
            target_speed = action[2]
            trigger_sequence = action[3]
            final_testcase_str += f"    {vehicle_name}.decelerate(trigger_sequence={trigger_sequence}, target_speed={target_speed})\n"
        elif action_type == "changeLane":
            target_lane = action[2]
            target_speed = action[3]
            trigger_sequence = action[4]
            final_testcase_str += f"    {vehicle_name}.changeLane(trigger_sequence={trigger_sequence}, target_lane={target_lane}, target_speed={target_speed})\n"

    return final_testcase_str

# 调用函数生成测试用例
if __name__ == '__main__':
    start_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    log_file_path = f'logs/{start_time}.log'
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s  %(filename)s : %(levelname)s  %(message)s',
                        datefmt='%Y-%m-%d %A %H:%M:%S',
                        filename=log_file_path,
                        filemode='w')
    logger = logging.getLogger(__name__)
    parser = Parser()
    with open("../../configs/config.yaml") as f:
        config = yaml.safe_load(f)
    # 循环生成测试用例
    for i in range(650):
        testcase_str = generate_testcase()
        test_case = parser.parse_testcase_string(testcase_str)
        simulation = Simulation()
        result, is_collision = simulation.run_test(test_case)
        logger.info("=== Evaluate Testcase === \n %s", testcase_str)
        logger.info("=== Evaluation Finished === \n Fitness: %s, Collision: %s", result, is_collision)
        if is_collision is True:
            logger.info("=== Find a collision ===")

