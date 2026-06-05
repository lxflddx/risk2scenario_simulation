# -*- coding: utf-8 -*-
import astunparse

from risk2scenario.core.testcase import TestCase
from risk2scenario.core.statement import ConstructorStatement, MethodStatement
import ast
import logging
import json

logger = logging.getLogger(__name__)


class Parser:
    @staticmethod
    def parse_testcase_string (testcase_str):
        testcase = TestCase()
        tree = ast.parse(testcase_str)

        for node in ast.walk(tree):

            if isinstance(node, ast.Assign) and isinstance(node.value, ast.Call):

                if isinstance(node.value.func, ast.Name) and node.value.func.id == 'NPC':

                    assignee = node.targets [0].id
                    class_name = node.value.func.id

                    args = {}
                    arg_bounds = {}

                    for keyword in node.value.keywords:

                        if isinstance(keyword.value, ast.List):

                            try:
                                value = [elt.n for elt in keyword.value.elts]
                                args [keyword.arg] = value
                                arg_bounds [keyword.arg] = value

                            except Exception as e:
                                logger.error(e)

                                args [keyword.arg] = [1, 1]
                                arg_bounds [keyword.arg] = [1, 1]

                        elif isinstance(keyword.value, ast.Constant):

                            args [keyword.arg] = keyword.value.value
                            arg_bounds [keyword.arg] = keyword.value.value

                    statement = ConstructorStatement(
                        testcase=testcase,
                        constructor_name=class_name,
                        assignee=assignee,
                        args=args,
                        arg_bounds=arg_bounds
                    )

                    statement.update_ast_node()
                    testcase.add_statement(statement)

            elif isinstance(node, ast.Expr) and isinstance(node.value, ast.Call):

                if isinstance(node.value.func, ast.Attribute):

                    callee = node.value.func.value.id
                    method_name = node.value.func.attr

                    args = {}
                    arg_bounds = {}

                    for keyword in node.value.keywords:

                        if isinstance(keyword.value, ast.List):

                            value = [elt.n for elt in keyword.value.elts]
                            args [keyword.arg] = value
                            arg_bounds [keyword.arg] = value

                        elif isinstance(keyword.value, ast.Constant):

                            args [keyword.arg] = keyword.value.value
                            arg_bounds [keyword.arg] = keyword.value.value

                    statement = MethodStatement(
                        testcase=testcase,
                        callee=callee,
                        method_name=method_name,
                        args=args,
                        arg_bounds=arg_bounds
                    )

                    statement.update_ast_node()
                    testcase.add_statement(statement)
        print(astunparse.unparse(ast.fix_missing_locations(testcase.update_ast_node())))

        return testcase

    @staticmethod
    def parse_testcase_json (data):

        testcase = TestCase()

        scenario = data.get("logical_scenario", data)

        # vehicle
        for vehicle in scenario.get("vehicles", []):

            assignee = vehicle ["vehicle_id"]

            args = {}
            arg_bounds = {}

            for key, value in vehicle.items():

                if key == "vehicle_id":
                    continue

                args [key] = value
                arg_bounds [key] = value

            statement = ConstructorStatement(
                testcase=testcase,
                constructor_name="NPC",
                assignee=assignee,
                args=args,
                arg_bounds=arg_bounds
            )

            statement.update_ast_node()
            testcase.add_statement(statement)

        action_mapping = {
            "Accelerate": "accelerate",
            "Decelerate": "decelerate",
            "LaneChanging": "changeLane"
        }

        # action
        for action in scenario.get("actions", []):

            callee = action ["vehicle_id"]

            method_name = action_mapping.get(action ["type"])

            if method_name is None:
                continue

            args = {}
            arg_bounds = {}

            for key, value in action.items():

                if key in ["vehicle_id", "type", "time_interval"]:
                    continue

                args [key] = value
                arg_bounds [key] = value

            statement = MethodStatement(
                testcase=testcase,
                callee=callee,
                method_name=method_name,
                args=args,
                arg_bounds=arg_bounds
            )

            statement.update_ast_node()
            testcase.add_statement(statement)
        print(astunparse.unparse(ast.fix_missing_locations(testcase.update_ast_node())))

        return testcase


# ==========================
# 示例输入测试
# ==========================
if __name__ == "__main__":
    parser = Parser()

    # --------------------------
    # 1. 字符串形式输入示例
    # --------------------------
    testcase_str = """
def testcase(): 
    vehicle1 = NPC(lane_id=[2, 2], offset=[45, 50], initial_speed=[30, 30])
    vehicle2 = NPC(lane_id=2, offset=20, initial_speed=0)
    vehicle3 = NPC(lane_id=[1, 2], offset=[10, 15], initial_speed=[0, 5])
    vehicle1.decelerate(target_speed=[0, 5], trigger_sequence=1)
    vehicle2.accelerate(target_speed=25, trigger_sequence=[2, 3])
"""
    print("\n=== 字符串解析示例 ===")
    testcase_from_str = parser.parse_testcase_string(testcase_str)

    # --------------------------
    # 2. JSON 形式输入示例（参数范围和具体值都支持）
    # --------------------------
    json_data = {
        "logical_scenario": {
            "vehicles": [
                {"vehicle_id": "ego", "lane_id": [2, 2], "offset": 0, "initial_speed": 30},
                {"vehicle_id": "V2", "lane_id": [1, 2], "offset": [10, 20], "initial_speed": [15, 25]},
                {"vehicle_id": "V3", "lane_id": 3, "offset": [5, 15], "initial_speed": 20}
            ],
            "actions": [
                {"vehicle_id": "ego", "type": "Accelerate", "target_speed": [35, 40], "trigger_sequence": 1,
                 "time_interval": [0, 1]},
                {"vehicle_id": "V2", "type": "Decelerate", "target_speed": [10, 15], "trigger_sequence": [2, 3],
                 "time_interval": [1, 2]},
                {"vehicle_id": "V3", "type": "LaneChanging", "target_lane": 2, "target_speed": 25,
                 "trigger_sequence": 3, "time_interval": [2, 3]}
            ]
        }
    }
    print("\n=== JSON 解析示例 ===")
    testcase_from_json = parser.parse_testcase_json(json_data)