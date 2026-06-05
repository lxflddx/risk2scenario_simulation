import logging
from datetime import datetime
import yaml
import openpyxl
import json

from risk2scenario.core.algorithm import Fuzzer
from risk2scenario.utils.my_parse import Parser

# ==========================
# 日志配置
# ==========================
start_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

log_file_path = f'logs/{start_time}.log'

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s  %(filename)s : %(levelname)s  %(message)s',
    datefmt='%Y-%m-%d %A %H:%M:%S',
    filename=log_file_path,
    filemode='w'
)

logger = logging.getLogger(__name__)

with open("configs/config.yaml") as f:
    config = yaml.safe_load(f)


# ==========================
# 读取Excel
# ==========================
def read_testcases_from_excel(excel_path):
    workbook = openpyxl.load_workbook(excel_path)
    sheet = workbook.active

    test_list = []

    for row in sheet.iter_rows(
            min_row=2,
            max_row=11,
            values_only=True):

        testcase_content = row[1]

        if testcase_content:
            test_list.append(testcase_content)

    return test_list


# ==========================
# 主程序
# ==========================
if __name__ == "__main__":

    excel_path = r"D:\pythonProject\LLM_Scenario\data\result_legend_gemma\my_results_gemma_new_2.xlsx"

    test_list = read_testcases_from_excel(excel_path)

    logger.info(f"file_path: {excel_path}")

    print("len(test_list):", len(test_list))

    my_parser = Parser()

    fuzzer = Fuzzer(config)

    all_results = []

    # ==========================
    # 逐个场景运行
    # ==========================
    for idx, testcase_content in enumerate(test_list, start=1):

        print(f"\n========== Running Case {idx} ==========")

        try:

            testcase_content = str(testcase_content).strip()

            # --------------------------
            # JSON格式
            # --------------------------
            if testcase_content.startswith("{"):

                print("Input Type: JSON")

                json_data = json.loads(testcase_content)

                logical_testcase = my_parser.parse_testcase_json(
                    json_data
                )

                testcase_type = "json"

            # --------------------------
            # 字符串格式
            # --------------------------
            else:

                print("Input Type: STRING")

                logical_testcase = my_parser.parse_testcase_string(
                    testcase_content
                )

                testcase_type = "string"

            # --------------------------
            # 模糊测试
            # --------------------------
            num, cs_list = fuzzer.loop(logical_testcase)

            result = {
                "case_id": idx,
                "input_type": testcase_type,
                "collision_num": num,
                "critical_scenarios": cs_list
            }

            all_results.append(result)

            print(f"collision_num = {num}")

            logger.info(result)

        except Exception as e:

            print(f"Case {idx} failed: {e}")

            logger.exception(e)

            all_results.append({
                "case_id": idx,
                "status": "failed",
                "error": str(e)
            })

    print("\n===================================")
    print("All testcases finished.")
    print("===================================")