# Risk2Scenario_SR_simulation
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

### 1. 安装 CARLA 0.9.15

- 从 [CARLA 官方 GitHub](https://github.com/carla-simulator/carla/releases/tag/0.9.15) 下载 Windows 版本。
- 解压到本地（例如 `C:\CARLA_0.9.15`）。
- 启动 CARLA 服务器：
  ```powershell
  cd C:\CARLA_0.9.15
  .\CarlaUE4.exe -quality-level=Low -fps=30
### 2. 准备输入数据
将上一阶段生成的逻辑场景测试用例（Excel 格式）放入 `data/` 目录。
### 3. 选择被测 Agent
打开 `risk2scenario/core/simulate.py`，在代码中设置所需的 Agent：
- `BasicAgent`（默认）
- `BehaviorAgent`
### 4. 运行主程序
在项目根目录下执行：
```powershell
python main.py
```
程序将执行：
- 读取 `data/` 中对应路径的Excel测试用例；
- 运行遗传算法进行多目标优化；
- 在 CARLA 仿真环境中评估每个场景；
- 日志保存到 `logs/`。
###5. 结果验证

若程序运行成功，仿真日志记录在 `logs/` 文件夹中，文件名如 `2026-06-02-13-23.log`。

**注意**：碰撞场景需要从日志中手工提取并验证。方法如下：

#### 1. 在日志中搜索关键场景

1. 打开 `logs/` 下的任意 `.log` 文件。
2. 搜索关键词（如 `find a collision` 等）。
3. 找到对应的具体场景测试用例。
4. 复制相关参数行，保存到文本文件备用。

#### 2. 回放关键场景

- 使用 `risk2scenario/utils/replay.py` 脚本。
- 将提取出的场景测试用例作为输入，运行仿真。
- 观察是否发生碰撞，并判断碰撞类型。

# 项目结构
```
risk_fuzz/
├── carla/                            # CARLA Python API 客户端及辅助脚本
├── configs/                          # 配置文件目录
│   ├── basic.json                    # 基础场景配置（道路、车辆参数等）
│   └── config.yaml                   # 遗传算法参数
├── data/                             # 数据目录
│   ├── legend/                       
│   ├── risk2Scenario_S2/             
│   └── risk2Scenario_SR/             
├── logs/                             # 仿真日志存放目录
├── risk2scenario/                    
│   ├── core/                         # 遗传算法与仿真核心模块
│   │   ├── algorithm.py              # 遗传算法主循环（选择、交叉、变异）
│   │   ├── carla_world.py            # CARLA 世界封装（生成车辆、车辆动作定义）
│   │   ├── DummyWorld.py             
│   │   ├── risk_chromosome.py        
│   │   ├── simulate.py               # 仿真代码
│   │   ├── statement.py              
│   │   └── testcase.py               
│   ├── random/                       # 随机基线
│   │   └── random_test.py            
│   └── utils/                        # 工具函数
│       ├── fnds.py                   # 适应度计算与 Pareto 前沿排序
│       ├── my_parse.py               # 测试用例语法解析
│       ├── replay.py                 # 场景回放
│       └── simulate_utils.py         
├── main.py                           # 程序入口
├── README.md                         
└── requirements.txt                  # Python 依赖包列表
