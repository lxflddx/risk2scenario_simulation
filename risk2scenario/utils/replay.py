import logging
import yaml
from datetime import datetime

from risk2scenario.core.algorithm import Fuzzer
from risk2scenario.core.risk_chromosome import Chromosome      # 请确保存在
from risk2scenario.utils.my_parse import Parser

start_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
log_file_path = f'logs/replay_{start_time}.log'
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s  %(filename)s : %(levelname)s  %(message)s',
                    datefmt='%Y-%m-%d %A %H:%M:%S',
                    filename=log_file_path,
                    filemode='w')
logger = logging.getLogger(__name__)

with open("configs/config.yaml") as f:
    config = yaml.safe_load(f)


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

if __name__ == "__main__":
    parser = Parser()
    fuzzer = Fuzzer(config=config)
    testcase = parser.parse_testcase_string(concrete_testcase_str)
    chrom = Chromosome(concrete_testcase=testcase)
    result = fuzzer.eval(chrom)
    logger.info(f"Evaluation result: {result}")
    print("Evaluation result:", result)