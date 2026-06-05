from __future__ import annotations

import time

from .risk_chromosome import Chromosome, ChromosomeFactory
import random
import numpy as np
import ast
import math
import astunparse
import logging
from .simulate import Simulation
from risk2scenario.utils import fnds

logger = logging.getLogger(__name__)


class Fuzzer:
    def __init__(self, config):

        self.config = config
        self.pop_size = 3
        self.max_iteration = config['max_iteration']
        self.selection_num = config["selection_num"]
        self.tournament_round = config["tournament_round"]
        self.crossover_rate = config["crossover_rate"]
        self.mutation_rate = config["mutation_rate"]
        self.chrom_factory = None
        self.logical_testcase = None
        self.last_elitism = None
        # self.sim = Simulation()
        self.pop = []
        self.collision = 0
        self.collision_scenario = []
        self.env_config = None

    # def initialize(self, logical_testcase):
    #     # 测试版
    #     self.pop_size = logical_testcase.size()
    #     print("=== Population Size: %d ===", self.pop_size)
    #     self.logical_testcase = logical_testcase
    #     self.pop = []
    #     self.chrom_factory = ChromosomeFactory(logical_testcase)
    #     for i in range(self.pop_size):
    #         chrom = self.chrom_factory.generate_random_chromosome()
    #         print("=== Generate Chromosome === \n %s", self.chrom2string(chrom))
    #         self.pop.append(chrom)
    #
    # def initialize(self):
    #     self.pop = []
    #     self.chrom_factory = ChromosomeFactory(self.logical_testcase)
    #     for i in range(self.pop_size):
    #         chrom = self.chrom_factory.generate_random_chromosome()
    #         self.pop.append(chrom)
    #     self.eval_pop()
    #     self.pop = self.get_survivals(n_survival=self.pop_size)
    #     for chrom in self.pop:
    #         chrom.fitness.append(-1.0)
    #     self.last_elitism = self.pop[0].clone()

    def initialize (self):
        self.pop = []
        self.chrom_factory = ChromosomeFactory(self.logical_testcase)

        # 开始计时
        start = time.perf_counter()

        for i in range(self.pop_size):
            chrom = self.chrom_factory.generate_random_chromosome()
            self.pop.append(chrom)

        end = time.perf_counter()
        elapsed = end - start
        print(f"生成 {self.pop_size} 个随机染色体耗时: {elapsed:.6f} 秒")

    def eval(self, testcase: Chromosome):
        """
        使用 run_test 函数评估单个测试用例的适应度，并检测是否发生碰撞。
        :param testcase: 要评估的测试用例。
        :return: (适应度, 是否碰撞)
        """
        logger.info("=== Evaluate Testcase === \n %s", self.chrom2string(testcase))
        fitness, is_collision = self.sim.run_test(testcase.testcase,env_config=self.env_config)
        logger.info("=== Evaluation Finished === \n Fitness: %s, Collision: %s", fitness, is_collision)
        if is_collision is True:
            logger.info("=== Find a collision ===")
        return fitness, is_collision

    def eval_pop(self, pop=None):
        """评价整个种群，在这里运行每一个测试用例，并记录碰撞的测试用例。"""
        pop = self.pop if pop is None else pop
        for chrom in pop:
            logger.info("=== Evaluate Chrom=== \n %s", self.chrom2string(chrom))
            print(f"test case: {self.chrom2string(chrom)}")
            # time.sleep(1)
            self.sim = Simulation()  # 重新初始化Simulation

            chrom.fitness, is_collision = self.sim.run_test(
                chrom.testcase,
                env_config=self.env_config
            )
            if is_collision is True:
                logger.info("=== Find a collision ===")
                self.collision += 1
                self.collision_scenario.append(self.chrom2string(chrom))  # 在这里把碰撞的测试用例记录下来
            logger.info("=== Evaluation Finished === \n fitness: %s", chrom.fitness)

    # def eval(self, chrom=None):
    #     # 只有在测试版的时候才会用到这个方法
    #     chrom.fitness = self.sim.run_test(chrom.testcase)

    def get_survivals(self, n_survival, pop=None):
        """
        这里其实就是帕累托选择，也就是多目标选择的过程，选择P+P'中最优的几个个体作为下一次迭代的种群
        :param n_survival:
        :param pop:
        :return:
        """
        pop = self.pop if pop is None else pop
        F = []
        for chrom in pop:
            F.append(chrom.fitness)
        logger.info("Pop F: %s", str(F))
        F = np.array(F).astype(float, copy=False)
        logger.info("Population Fitness: %s", str(F))
        survivors = []
        fronts = fnds.fast_non_dominated_sort(F)
        logger.info("Fronts: %s", str(fronts))
        for k, front in enumerate(fronts):
            crowding_of_front = fnds.calc_crowding_distance(F[front, :])
            for j, i in enumerate(front):
                pop[i].rank = k
                pop[i].crowding = crowding_of_front[j]

            if len(survivors) + len(front) > n_survival:
                P = np.random.permutation(len(crowding_of_front))
                I = np.argsort(crowding_of_front[P], kind='quicksort')
                I = P[I]
                I = np.flip(I, axis=0)
            else:
                I = np.arange(len(front))

            survivors.extend(front[I])
        logger.info("Survivors: %s", str(survivors))
        return [pop[i] for i in survivors]

    def select(self, num, max_round, pop=None):
        """
       锦标赛选择，选进行交叉和变异的个体（通过锦标赛随机为每一次交叉和变异选择父代）
        """
        pop = self.pop if pop is None else pop
        selection = []
        for _ in range(num):
            # 随机选择一个初始个体作为冠军
            new_idx = random.randint(0, len(pop) - 1)
            winner_idx = new_idx
            round = 0
            while round < max_round:
                # 随机选择一个与冠军不同的个体作为竞争者
                new_idx = random.randint(0, len(pop) - 1)
                selected = pop[new_idx]
                if selected.fitness < pop[winner_idx].fitness:
                    winner_idx = new_idx
                round += 1
            selection.append(pop[winner_idx])
        return selection

    def crossover(self, chrom1: Chromosome, chrom2: Chromosome):
        clone1 = chrom1.clone()
        clone2 = chrom2.clone()
        chrom1.crossover(clone2)
        chrom2.crossover(clone1)

    def update_fitness(self, pop):
        last_elitism_arg = []
        for statement in self.last_elitism.testcase.statements:
            last_elitism_arg.extend(list(statement.args.values()))
        for chrom in pop:
            chrom_arg = []  # 存储种群中每个个体的参数列表
            for statement in chrom.testcase.statements:
                chrom_arg.extend(list(statement.args.values()))
            # 如果两个参数列表长度不相等，说明两个个体不适合进行进化，直接给个体设置适应度为-1
            if len(last_elitism_arg) != len(chrom_arg):
                logger.info(self.chrom2string(self.last_elitism))
                logger.info(self.chrom2string(chrom))
                logger.error("The two param list must be of equal length")
                chrom.fitness.append(-1.0)
                continue
            # 计算两个个体的参数差异的平方和，作为适应度
            squared_differences = [(a - b) ** 2 for a, b in zip(last_elitism_arg, chrom_arg)]
            sum_of_squared_differences = sum(squared_differences)
            distance = math.sqrt(sum_of_squared_differences)
            chrom.fitness.append(-1.0 * distance)  # maximize the diversity希望增大个体和种群中最优个体之间的差异

    def evolve(self):
        """
        遗传算法进化
        """
        new_pop = []
        # new_pop.extend(self.elitism())
        while len(new_pop) < self.pop_size:
            parent1 = self.select(num=self.selection_num, max_round=self.tournament_round)[0]
            parent2 = self.select(num=self.selection_num, max_round=self.tournament_round)[0]

            offspring1 = parent1.clone()
            offspring2 = parent2.clone()

            logger.info("parent 1: %s", self.chrom2string(parent1))
            logger.info("parent 2: %s", self.chrom2string(parent2))

            if random.random() <= self.crossover_rate:
                logger.info("====== Start Crossover ======")
                self.crossover(offspring1, offspring2)

            logger.info("offspring 1 after crossover: %s", self.chrom2string(offspring1))
            logger.info("offspring 2 after crossover: %s", self.chrom2string(offspring2))

            offspring1.mutate()


            logger.info("offspring 1 after mutation: %s", self.chrom2string(offspring1))

            offspring2.mutate()
            logger.info("offspring 2 after mutation: %s", self.chrom2string(offspring2))


            new_pop.append(offspring1)
            new_pop.append(offspring2)

        self.eval_pop(new_pop)  # 此处运行仿真，评估种群中每一个个体的适应度
        self.update_fitness(new_pop)
        pop = self.pop + new_pop
        self.pop = self.get_survivals(self.pop_size, pop)
        self.last_elitism = self.pop[0].clone()  # 适应度最高的个体是最优的，其他个体都要在下一次update_fitness时计算和它的距离，然后更新适应度分数
        logger.info("===Updated Population=== \n")
        for p in self.pop:
            logger.info("fitness: %s", str(p.fitness))
        # logger.info("The best individual: %s \r\n its fitness score is %s",
        #                  self.chrom2string(self.population[0]), str(self.population[0].fitness))

    def loop(self, logical_testcase,env_config=None):
        self.logical_testcase = logical_testcase
        self.pop_size = logical_testcase.size()
        self.env_config = env_config or {}
        logger.info("Pop size: %d", self.pop_size)
        # logger.info("Pop size: %d", 3)
        self.initialize()
        iteration = 0
        is_need_to_mutate = False
        while iteration < self.max_iteration:
            logger.info("===Iteration %d===", iteration)
            self.evolve()
            iteration += 1
            # if self.collision == 0:
            #         # self.pop[0].fitness[0] > 6.0:
            #     is_need_to_mutate = True
            #     break
        logger.info("This loop has %d iterations and found %d collisions", iteration, self.collision)
        num = self.collision
        cs_list = self.collision_scenario
        self.collision = 0
        self.collision_scenario = []
        return num, cs_list

    def elitism(self):
        elite = []
        for idx in range(self.config["elite"]):
            elite.append(self.pop[idx].clone())
        return elite

    @staticmethod
    def chrom2string(chrom):
        return astunparse.unparse(ast.fix_missing_locations(chrom.testcase.update_ast_node()))

    # def evolution(self, num_mutations):
    # """
    # 我自己写的只有变异的算法
    # """
    #     results = []
    #     for i in range(num_mutations):
    #         logger.info("=== Generate Offspring === ", i + 1)
    #         mutated_testcase = self.chrom.clone()
    #         mutated_testcase.mutate()
    #         logger.info("=== Mutated Testcase === \n %s", self.chrom2string(mutated_testcase))
    #         # fitness, is_collision = self.eval(mutated_testcase)
    #         # results.append((mutated_testcase, fitness, is_collision))
    #     return results

    # def run(self, num_mutations):
    #     """
    #     执行变异和评估流程。
    #     :param num_mutations: 变异次数。
    #     """
    #     results = self.evolution(num_mutations)
    #     logger.info("=== Summary of Mutations ===")
    #     for i, (testcase, fitness, is_collision) in enumerate(results):
    #         logger.info("=== Mutation %d ===", i + 1)
    #         logger.info("Testcase: %s", self.chrom2string(testcase))
    #         # logger.info("Fitness: %s, Collision: %s", fitness, is_collision)

