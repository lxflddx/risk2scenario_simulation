import logging
from datetime import datetime
import json
import yaml
import ctypes
import openpyxl
from risk2scenario.core.algorithm import Fuzzer
from risk2scenario.utils.my_parse import Parser


start_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
log_file_path = f'logs/{start_time}.log'
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s  %(filename)s : %(levelname)s  %(message)s',
                    datefmt='%Y-%m-%d %A %H:%M:%S',
                    filename=log_file_path,
                    filemode='w')
logger = logging.getLogger(__name__)
with open("configs/config.yaml") as f:
    config = yaml.safe_load(f)


def read_testcases_from_excel(excel_path):
    workbook = openpyxl.load_workbook(excel_path)
    sheet = workbook.active
    test_list = []

    for row in sheet.iter_rows(min_row=2, values_only=True):
        testcase_id = row[0]   # 第一列：TC_01, TC_02 ...
        testcase_str = row[1]  # 第二列：testcase 字符串

        if testcase_id and testcase_str:
            test_list.append({
                "testcase_id": str(testcase_id).strip(),
                "testcase_str": testcase_str
            })

    return test_list


def load_env_configs (json_path):
    with open(json_path, "r", encoding="utf-8") as f:
        return json.load(f)


# if __name__ == "__main__":
#     # 定义具体的测试用例字符串
#     parser = Parser()
#     concrete_testcase_str = """
# def testcase(self):
#     vehicle1 = NPC(lane_id=3, offset=15.0, initial_speed=9.802572945189173)
#     vehicle2 = NPC(lane_id=3, offset=0.0, initial_speed=27.571887129326882)
#     ego = NPC(lane_id=3, offset=30.0, initial_speed=26.142930387200572)
#     vehicle1.decelerate(target_speed=6.662522574199049, trigger_sequence=1)
#     vehicle1.changeLane(target_lane=2, target_speed=2.4245320169811624, trigger_sequence=2)
#     vehicle2.accelerate(target_speed=49.84154727816018, trigger_sequence=3)
#     vehicle1.accelerate(target_speed=34.00498520564464, trigger_sequence=4)
#     vehicle1.decelerate(target_speed=0.27781828538521747, trigger_sequence=5)
#     vehicle2.decelerate(target_speed=0.9965727275480045, trigger_sequence=4)
#     """
    # test_list = ["""def testcase(self):
    # ego=NPC(lane_id=[3,3],offset=[25,35],initial_speed=[5.0,15.0])
    # vehicle2=NPC(lane_id=[1,2,3],offset=[15,25],initial_speed=[20.0,40.0])
    # vehicle3=NPC(lane_id=[1,2,3],offset=[15,25],initial_speed=[20.0,30.0])
    # vehicle3.changeLane(target_lane=[1,2,3],target_speed=[25.0,35.0],trigger_sequence=[1],time_interval=[1,5])
    # vehicle3.changeLane(target_lane=[1,2,3],target_speed=[20.0,30.0],trigger_sequence=[2],time_interval=[1,5])
    # vehicle2.changeLane(target_lane=[1,2,3],target_speed=[20.0,40.0],trigger_sequence=[2],time_interval=[1,5])
    #     """]
    # folder_path = "D:\\pythonProject\\LLM_Scenario\\data\\my_results"
    # test_list = process_testcases(folder_path)
    # excel_path = r"D:\pythonProject\LLM_Scenario\data\without_risk_to_scenario\without_risk_to_scenario_4.xlsx"
if __name__ == "__main__":


    excel_path = r"D:\pythonProject\LLM_Scenario\data\risk_combine\risk_combine_1.xlsx"
    env_json_path = r"D:\pythonProject\LLM_Scenario\env_configs_14.json"

    test_list = read_testcases_from_excel(excel_path)
    env_configs = load_env_configs(env_json_path)
    # logger.info("without time internal town04")
    # logger.info("without time internal town06")
    # logger.info("with time internal town04")
    logger.info("with time internal town04")
    logger.info(f"file_path: {excel_path}")
    print("len(test_list):", len(test_list))

    # 只运行这一批，分地图运行
    # selected_ids = {"TC_01", "TC_05", "TC_08", "TC_10", "TC_12", "TC_13"}
    selected_ids = {"TC_02", "TC_03", "TC_04", "TC_06", "TC_07", "TC_09","TC_11","TC_14"}



    logger.info(f"selected_ids: {selected_ids}")
    my_parser = Parser()


    fuzzer = Fuzzer(config)

    for item in test_list:
        testcase_id = item ["testcase_id"]
        testcase_str = item ["testcase_str"]

        # 不在本轮运行范围内就跳过
        if testcase_id not in selected_ids:
            continue

        # 取对应环境配置
        env_config = env_configs.get(testcase_id)
        if env_config is None:
            logger.warning(f"Missing env_config for {testcase_id}, skip.")
            print(f"Missing env_config for {testcase_id}, skip.")
            continue

        print(f"\n=== Running {testcase_id} ===")
        print(f"env_config: {env_config}")
        logger.info(f"Running {testcase_id}, env_config={env_config}")

        logical_testcase = my_parser.parse_testcase_string(testcase_str)
        num, cs_list = fuzzer.loop(logical_testcase, env_config=env_config)

        data = {}
        data ["testcase_id"] = testcase_id
        data ["env_config"] = env_config
        data ["collision_num"] = num
        data ["critical_scenarios"] = cs_list

        print(f"{testcase_id} collision_num: {num}")
        logger.info(f"{testcase_id} collision_num: {num}")
        logger.info(f"{testcase_id} critical_scenarios: {cs_list}")
    ctypes.windll.user32.MessageBoxW(
        0,
        "本轮测试已全部运行完毕。",
        "运行完成",
        0x400
    )

