from risk2scenario.core.testcase import TestCase
from risk2scenario.core.statement import ConstructorStatement, MethodStatement
import ast
import astunparse
import logging

logger = logging.getLogger(__name__)


class Parser:
    @staticmethod
    def parse_testcase_string(testcase_str):
        """
        将字符串形式的测试用例解析为 TestCase 对象。

        Args:
            testcase_str (str): 字符串形式的测试用例。

        Returns:
            TestCase: 解析后的测试用例对象。
        """
        testcase = TestCase()
        tree = ast.parse(testcase_str)
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign) and isinstance(node.value, ast.Call):
                if isinstance(node.value.func, ast.Name) and node.value.func.id == 'NPC':
                    # 解析构造函数调用
                    assignee = node.targets[0].id
                    class_name = node.value.func.id
                    args = {}
                    arg_bounds = {}
                    for keyword in node.value.keywords:
                        if isinstance(keyword.value, ast.List):
                            try:
                                arg_bounds[keyword.arg] = [elt.n for elt in keyword.value.elts]
                                args[keyword.arg] = [elt.n for elt in keyword.value.elts]
                            except Exception as e:
                                logger.error(e)
                                arg_bounds[keyword.arg] = [1, 1]
                                args[keyword.arg] = [1, 1]
                        elif isinstance(keyword.value, ast.Constant):
                            arg_bounds[keyword.arg] = keyword.value.value
                            args[keyword.arg] = keyword.value.value

                    # 创建 ConstructorStatement 并添加到 TestCase
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
                    # 解析方法调用
                    callee = node.value.func.value.id
                    method_name = node.value.func.attr
                    args = {}
                    arg_bounds = {}
                    for keyword in node.value.keywords:
                        if isinstance(keyword.value, ast.List):
                            arg_bounds[keyword.arg] = [elt.n for elt in keyword.value.elts]
                            args[keyword.arg] = [elt.n for elt in keyword.value.elts]
                        elif isinstance(keyword.value, ast.Constant):
                            arg_bounds[keyword.arg] = keyword.value.value
                            args[keyword.arg] = keyword.value.value

                    # 创建 MethodStatement 并添加到 TestCase
                    statement = MethodStatement(
                        testcase=testcase,
                        callee=callee,
                        method_name=method_name,
                        args=args,
                        arg_bounds=arg_bounds
                    )
                    statement.update_ast_node()
                    testcase.add_statement(statement)

        # 打印生成的测试用例的 AST
        print(astunparse.unparse(ast.fix_missing_locations(testcase.update_ast_node())))
        return testcase


if __name__ == "__main__":
    # 示例：解析字符串形式的测试用例
    parser = Parser()
    testcase_str = """
def testcase(): 
    vehicle1 = NPC(lane_id=2, offset=20, initial_speed=30) 
    vehicle2 = NPC(lane_id=2, offset=20, initial_speed=0) 
    vehicle3 = NPC(lane_id=2, offset=20, initial_speed=0) 
    vehicle4 = NPC(lane_id=2, offset=20, initial_speed=0) 
    vehicle1.decelerate(target_speed=0, trigger_sequence=1) 
"""
    # 确保 testcase_str 是字符串类型
    if not isinstance(testcase_str, str):
        raise TypeError("Input testcase_str must be a string.")

    # 解析测试用例字符串
    testcase = parser.parse_testcase_string(testcase_str)
