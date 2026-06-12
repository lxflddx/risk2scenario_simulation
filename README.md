# Risk2Scenario_CT_simulation
## 方法概述
本框架从上一阶段生成的逻辑场景测试用例出发，在仿真环境中运行场景，进行多目标优化，生成场景。
## 依赖环境
| 类别       | 配置                                   |
|------------|----------------------------------------|
| CPU        | Intel Core i9-10900K @ 3.70 GHz        |
| RAM        | 32 GB                                   |
| 显卡       | NVIDIA RTX 4000                         |
| 操作系统   | Windows 10                              |
| 仿真器     | CARLA 0.9.15                            |
| 被测系统   | CARLA Basic Agent / CARLA Behavior Agent |
| Python     | 3.8+                                    |
## 使用步骤
### 1. 环境安装

```bash
# 创建虚拟环境（可选）
python -m venv .venv
.venv\Scripts\activate

# 安装 Python 依赖
pip install -r requirements.txt
```

### 2. 安装 CARLA 0.9.15

- 从 [CARLA 官方 GitHub](https://github.com/carla-simulator/carla/releases/tag/0.9.15) 下载 Windows 版本。
- 解压到本地（例如 `C:\CARLA_0.9.15`）。
- 启动 CARLA 服务器：
  ```powershell
  CarlaUE4.exe -preferNvidia -quality-level=Low -benchmark -fps=15 -windowed -ResX=600 -ResY=480

### 3. 准备输入数据
将上一阶段生成的逻辑场景测试用例（Excel 格式）放入 `data/` 目录。
### 4. 选择被测 Agent
打开 `risk2scenario/core/simulate.py`，在代码中设置所需的 Agent：
- `BasicAgent`（默认）
- `BehaviorAgent`
### 5. 运行主程序
在项目根目录下执行：
```powershell
python main.py
```
程序将执行：
- 读取 `data/` 中对应路径的Excel测试用例；
- 运行遗传算法进行多目标优化；
- 在 CARLA 仿真环境中评估每个场景；
- 日志保存到 `logs/`。

**时间敏感消融实验**：若需去除时间敏感交互建模进行消融实验，在 `configs/config.yaml` 中将 `time_interval` 参数改为 `[5, 5]`，固定间隔，不再动态变化。
### 6. 结果验证

若程序运行成功，仿真日志记录在 `logs/` 文件夹中，文件名如 `2026-06-02-13-23.log`。

**注意**：碰撞场景需要从日志中手工提取并验证。方法如下：

#### 1. 在日志中搜索关键场景
- 打开 `logs/` 下的任意 `.log` 文件。
- 搜索关键词（如 `find a collision` 等）。
- 找到对应的具体场景测试用例。
- 复制相关参数行，保存到文本文件备用。

#### 2. 回放关键场景

- 使用 `risk2scenario/utils/replay.py` 脚本。
- 将提取出的场景测试用例作为输入，运行仿真。
- 观察是否发生碰撞，并判断碰撞类型。

#### 3. 有效碰撞场景筛选标准
满足以下条件的场景才被认定为有效碰撞场景：
- 场景回放过程中实际发生碰撞；
- 碰撞由主车决策或控制失效导致；
- 主车已采取最大制动但仍无法避免的碰撞不计入有效碰撞；
- 主车无主要责任的碰撞（如主车被npc车辆追尾）不计入有效碰撞。

# 项目结构
```
risk_fuzz/
├── carla/                            # CARLA Python API
├── configs/                          # 配置文件
│   ├── basic.json                    # 基础场景配置
│   └── config.yaml                   # 遗传算法参数
├── data/                             # 种子场景测试用例（.xlsx）
│   ├── combine_gemma_12b/            # 小参数大模型消融实验种子场景数据
│   ├── risk2Scenario_C2/             # C2消融实验种子场景数据
│   └── risk2Scenario_CT/             # CT增强种子场景数据
├── logs/                             # 仿真日志存放目录
├── results_logs/                     # 整理后的最终结果日志
├── risk2scenario/                    # 场景生成
│   ├── core/                         # 遗传算法与仿真核心模块
│   │   ├── algorithm.py              # 遗传算法
│   │   ├── carla_world.py            # CARLA世界封装，实现车辆动作
│   │   ├── DummyWorld.py             
│   │   ├── risk_chromosome.py        
│   │   ├── simulate.py               # 仿真执行
│   │   ├── statement.py              
│   │   └── testcase.py               
│   └── utils/                        # 工具函数
│       ├── fnds.py                   # 适应度计算与 Pareto 前沿排序
│       ├── my_parse.py               # 测试用例语法解析
│       ├── replay.py                 # 场景回放
│       └── simulate_utils.py         
├── main.py                           # 程序入口
├── README.md                         
└── requirements.txt            
