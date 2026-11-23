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
colcon build --packages-select shelf_scene
source install/setup.bash
ros2 run shelf_scene publish_scene --ros-args -p seed:=7
```
RViz 的 Fixed Frame 设为 world，添加 MarkerArray，选择 /shelf/markers，Durability 设为 Transient Local。/planning_scene 发布一次带相同几何的增量，订阅者应在启动节点前就绪，或使用 transient_local 获取缓存。每个种子对应一个独立试验。执行抓取时需由执行器管理附着与世界物体的切换。
