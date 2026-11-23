# 七自由度机械臂货架取放
目标是在三面封闭的货架内抓取盒子，并在执行前验证带载撤出路径。采用 ROS 2 Humble、MoveIt 2 和 Gazebo Classic。拟用 Panda 七轴单臂作为冗余机械臂模型，使用仿真真值位姿。

## 实施路线与验收
1. 场景输入。统一用米和 world 坐标系描述架板、目标盒、邻近障碍与料盘底板。按种子复现布局；发布可在 RViz 中查看的场景以及可接入 MoveIt 的规划场景增量。
2. 固定姿态基线。接入 Panda URDF/SRDF、关节控制器及 Gazebo 夹爪。由同一份几何生成物理场景。实现预抓取、直线接近、合爪、附着、撤出、移动、释放。以物体静止并落在盘内验收。
3. 多候选规划。枚举抓取方向与 IK 初值，对整臂检查限位、碰撞、接近可行性。按最小间隙和关节位移排序，记录每个被拒候选的原因。
4. 带载预检。在隔离的规划场景中附着盒子，验算撤离路径；失败后试下一个候选。执行层根据夹爪开度与物体相对位移识别抓空，重新观测，限制重试次数。
5. 对照实验。三种策略共享场景种子、超时与初始状态。按格口宽度与深度分组，统计入盘成功率、抓住后的撤出失败率、规划耗时 P95。保存配置、逐次结果和典型失败回放。

## 当前运行入口
在仓库根目录用 Bash 执行：
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to shelf_scene
source install/setup.bash
ros2 run shelf_scene publish_scene --ros-args -p seed:=7
```
RViz 的 Fixed Frame 设为 world，添加 MarkerArray，选择 /shelf/markers，Durability 设为 Transient Local。/planning_scene 发布一次带相同几何的增量，订阅者应在启动节点前就绪，或使用 transient_local 获取缓存。每个种子对应一个独立试验。执行抓取时需由执行器管理附着与世界物体的切换。

在已加载工作区环境的终端中，以下命令用同一个种子启动 Gazebo Classic 物理场景。货架和料盘固定，目标盒质量为 0.1 kg，可受重力和接触力作用。

```bash
ros2 run shelf_scene gazebo_world --seed 7 > /tmp/shelf-7.world
gzserver /tmp/shelf-7.world
```

要同时启动 Panda、货架及关节控制器，在 ROS 2 Humble 环境下执行：

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to shelf_scene
source install/setup.bash
ros2 launch shelf_scene baseline.launch.py seed:=7
```

该入口使用 MoveIt Resources 的 Panda 几何与初始关节角、Franka FER 与夹爪惯性参数，以及 Gazebo 的 ros2_control 接口。启动后可通过 `ros2 control list_controllers` 查看手臂、夹爪和关节状态控制器。

启动基线后，在已加载工作区的另一终端执行 `ros2 run shelf_scene fixed_grasp`。节点读取 Gazebo 真值位姿，以固定方向接近盒子，两指分别用限力控制分步闭合。Gazebo 插件要求左右指在同一物理步均与目标发生碰撞才建立固定约束，开爪后解除约束。这是仿真抓持近似。MoveIt 同时将盒子附着到手掌以检查带载轨迹。执行器用 Gazebo 真值检查 35 mm 试抬，直线路径不完整时停止。释放后检查盒子投影是否完全落入盘内、底部是否接触盘面，并要求线速度和角速度连续 0.5 秒仿真时间低于阈值。

`baseline.launch.py` 的 `width` 是格口净宽（默认 0.36 m，范围 0.24–0.50 m），`depth` 是货架外深（默认 0.32 m，范围 0.23–0.40 m）。Gazebo、MoveIt 和 RViz 共享这两个参数及场景种子。

## 三种策略与对照实验

在相同的仿真启动命令下，执行 `ros2 run shelf_scene fixed_grasp --ros-args -p strategy:=precheck`。支持以下策略：

| 参数 | 抓取规划 |
| --- | --- |
| `fixed` | 固定从架前接近，使用当前状态求预抓取 IK。 |
| `multi` | 枚举正面、盒子朝向及其左右偏转方向，各用三个肘部初值求 IK。检查接近轨迹的关节限位、整臂碰撞和完整性，优先选沿途对环境最小间隙大的候选，再比较关节累计位移。 |
| `precheck` | 在相同候选检查基础上，在独立的 MoveIt 场景副本里附着目标盒，以最大 5 mm 步长验证抬升和带载撤出的 IK、关节连续性与碰撞。 |

三种策略共用抓持确认、释放和入盘验收。抓空时重新观察盒子，更新规划场景，最多再尝试一次。程序输出的 `TRIAL_RESULT` 包含失败阶段、候选拒绝原因、尝试次数及规划耗时。

加载已构建的工作区后，可从仓库根目录运行配对实验：

```bash
python3 scripts/compare.py log/study --seeds 7 8 --widths 0.30 0.36 --depths 0.28 0.32
```

输出目录须为新目录。每个格口尺寸与种子组合均从同一机器人初始姿态新建三次仿真。默认每次执行限时 120 秒，启动单独限时 45 秒。`config.json` 保存实验参数，`trials.jsonl` 保存逐次结果，`summary.json` 按净宽、外深和策略汇总。入盘率以全部试验为分母，撤出失败率以通过试抬的次数为分母。P95 用最近秩法，样本是每次执行中 IK、笛卡尔规划、候选检查和关节路径规划请求的累计墙钟耗时，包含服务通信，同时给出有完整计时的样本数。每次试验均保存执行日志和 ROS bag。可用 `ros2 bag info <试验目录>/replay` 查看主题，用 `ros2 bag play <试验目录>/replay` 回放仿真物体位姿、关节、TF 和规划场景。

### 首轮实测

种子 7、8 与两档净宽、两档外深组合成 8 个配对场景。固定姿态、多候选、带载预检各完成 8 次试验，稳定入盘次数分别为 2、3、2。净宽 0.30 m 的 12 次试验均在接近阶段终止。净宽 0.36 m、外深 0.28 m、种子 8 时，仅多候选策略完成入盘。7 次通过试抬的试验均完成撤出；本轮样本未观察到带载预检降低撤出失败率的效果。

[实验配置](experiments/paired-shelf-trials/config.json) · [逐次记录与拒绝原因](experiments/paired-shelf-trials/trials.jsonl) · [按格口尺寸汇总的三项指标](experiments/paired-shelf-trials/summary.json)

回放净宽 0.30 m 场景下固定姿态接近失败的样例：

```bash
ros2 bag play experiments/paired-shelf-trials/w0.3-d0.28-s7-fixed/replay
```
