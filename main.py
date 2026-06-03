import logging
from datetime import datetime
import yaml
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
    for row in sheet.iter_rows(min_row=2, max_row=11,values_only=True):  # 第一行是表头
        testcase_str = row[1]  # 测试用例字符串在第二列
        if testcase_str:
            test_list.append(testcase_str)
    return test_list

if __name__ == "__main__":
    # 定义具体的测试用例字符串
    my_parser = Parser()
    concrete_testcase_str = """
def testcase(self):
    vehicle1 = NPC(lane_id=3, offset=15.0, initial_speed=9.802572945189173)
    vehicle2 = NPC(lane_id=3, offset=0.0, initial_speed=27.571887129326882)
    ego = NPC(lane_id=3, offset=30.0, initial_speed=26.142930387200572)
    vehicle1.decelerate(target_speed=6.662522574199049, trigger_sequence=1)
    vehicle1.changeLane(target_lane=2, target_speed=2.4245320169811624, trigger_sequence=2)
    vehicle2.accelerate(target_speed=49.84154727816018, trigger_sequence=3)
    vehicle1.accelerate(target_speed=34.00498520564464, trigger_sequence=4)
    vehicle1.decelerate(target_speed=0.27781828538521747, trigger_sequence=5)
    vehicle2.decelerate(target_speed=0.9965727275480045, trigger_sequence=4)
    """
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
    # excel_path = r"D:\pythonProject\LLM_Scenario\data\without_risk_to_scenario\without_risk_to_scenario_2.xlsx"
    excel_path = r"D:\pythonProject\LLM_Scenario\data\result_legend_gemma\my_results_gemma_new_2.xlsx"
    test_list = read_testcases_from_excel(excel_path)
    logger.info(f"file_path: {excel_path}")
    print("test_list:", test_list)
    print("len(test_list):", len(test_list))

    fuzzer = Fuzzer(config)

    # 遍历测试用例并执行模糊测试
    for testcase_str in test_list:
        logical_testcase = my_parser.parse_testcase_string(testcase_str)
        num, cs_list = fuzzer.loop(logical_testcase)
        # record_path = "data/results/" + str(id) + '.json'
        data = {}
        data["collision_num"] = num
        data["critical_scenarios"] = cs_list
        # with open(record_path, 'w') as f:
        #     json.dump(data, f, indent=4)

    print(data["critical_scenarios"])







