# 机器人足球多角色协同策略系统

<div align="center">

![Competition](https://img.shields.io/badge/RoboCup-Middle%20Size%20Simulation-red)
![ROS](https://img.shields.io/badge/ROS-Kinetic-blue)
![C++](https://img.shields.io/badge/C++-11-orange)
![Status](https://img.shields.io/badge/Status-Competition%20Ready-green)

**基于ROS的5v5机器人足球分布式协同决策与控制系统**
![演示动图](demo.gif)
</div>

---

## 📋 目录

- [项目简介](#项目简介)
- [开发背景](#开发背景)
- [核心架构](#核心架构)
- [功能模块](#功能模块)
- [技术亮点](#技术亮点)
- [代码结构](#代码结构)
- [运行环境](#运行环境)
- [快速开始](#快速开始)
- [开发历程](#开发历程)

---

## 🎯 项目简介

本项目是我参加**中国机器人大赛暨RoboCup世界杯中国赛 中型组仿真赛**开发的完整机器人足球控制策略系统。在赛方提供的ROS+Gazebo仿真平台基础上，我独立实现了包含**策略决策、角色分配、行为规划、运动控制**在内的完整控制系统，共计**18个核心模块、超过3000行代码**。

### 主要成果

✅ **分布式多角色系统**：实现4种角色（进攻、协助、防守、守门）的动态分配与协同  
✅ **智能决策引擎**：基于有限状态机的实时决策系统，15ms响应周期  
✅ **传球协作机制**：静态传球与动态传球的完整实现  
✅ **精确运动控制**：全向移动平台的路径规划与避障  
✅ **专项技能实现**：带球、射门、拦截、守门等10+种足球技能  

---

## 🏆 开发背景

### 比赛介绍

- **比赛名称**：中国机器人大赛暨RoboCup世界杯中国赛 中型组仿真赛
- **比赛形式**：5v5机器人足球对抗赛
- **技术要求**：基于ROS框架开发分布式机器人控制策略
- **开发范围**：参赛队伍需要在赛方提供的仿真平台基础上，独立实现 `robot_code/nubot_control/src/` 目录下的所有策略代码

### 技术挑战

🎯 **实时性要求**：控制周期仅15毫秒，需要完成感知、决策、规划和控制全流程  
🤖 **多机协同**：5个机器人需要实时协调，避免冲突和重复决策  
⚽ **动态对抗**：面对不同风格的对手需要灵活调整策略  
🔄 **状态复杂度**：需要处理开球、定位球、点球等10+种比赛状态  

---

## 🏗️ 核心架构

### 系统架构图

```
┌─────────────────────────────────────────────────────────┐
│                    NuBot Control                         │
│                   (主控制器)                              │
└───────────┬────────────────────────────────┬────────────┘
            │                                │
    ┌───────▼────────┐              ┌───────▼────────┐
    │   Strategy     │              │     Plan       │
    │  (策略层)       │◄────────────►│   (规划层)      │
    └───────┬────────┘              └───────┬────────┘
            │                                │
    ┌───────▼────────┐              ┌───────▼────────┐
    │ Role Assignment│              │   Behaviour    │
    │  (角色分配)     │              │  (行为控制)     │
    └───────┬────────┘              └───────┬────────┘
            │                                │
    ┌───────┴────────────────────────────────┴────────┐
    │              角色执行层                          │
    ├──────────┬──────────┬──────────┬───────────────┤
    │ Active   │ Assist   │ Passive  │ Midfield     │
    │  Role    │  Role    │  Role    │  Role        │
    │(主攻手)   │(协助者)   │(防守者)   │(中场)        │
    └──────────┴──────────┴──────────┴───────────────┘
```

### 分层设计

1. **控制层** (`nubot_control.cpp`)：系统主入口，协调各模块运行
2. **策略层** (`strategy.cpp`)：高层决策，角色分配与策略选择
3. **规划层** (`plan.cpp`, `subtargets.cpp`)：路径规划，子目标生成
4. **行为层** (`behaviour.cpp`)：具体动作执行与运动控制
5. **角色层**：各角色的专项策略实现

---

## 🎯 功能模块

### 1. 核心控制系统（Core Control）

#### 📄 `nubot_control.cpp` - 主控制器
**核心功能：**
- 15ms高频控制循环
- ROS消息收发与状态同步
- 比赛状态机管理
- 全局信息融合

**关键代码：**
```cpp
void loopControl(const ros::TimerEvent& event)
{
    // 更新世界模型
    updateWorldModel();
    
    // 根据比赛模式执行策略
    switch(match_mode_) {
        case STOPROBOT:   stopRobot(); break;
        case STARTROBOT:  normalGame(); break;
        case KICKOFF:     kickoffStrategy(); break;
        // ... 更多比赛状态
    }
    
    // 执行运动控制
    setEthercatCommand();
    publishStrategyInfo();
}
```

#### 📄 `main.cpp` - 程序入口
- ROS节点初始化
- 多线程管理
- 异常处理

---

### 2. 策略决策系统（Strategy System）

#### 📄 `strategy.cpp` - 策略引擎
**实现功能：**
- 全局战术选择
- 传球时机判断
- 角色行为协调
- 状态转换控制

**核心策略：**
```cpp
void Strategy::execute()
{
    // 传球策略
    if(shouldPass()) {
        executePassStrategy();
    }
    // 射门策略
    else if(canShoot()) {
        executeShootStrategy();
    }
    // 角色执行
    else {
        executeRoleStrategy();
    }
}
```

#### 📄 `role_assignment.cpp` - 角色分配器
**分配算法：**
- 距离优先原则：最近的机器人成为主攻手
- 区域防守策略：根据场上位置分配防守角色
- 动态调整机制：根据局势变化实时切换角色

**角色类型：**
- **ActiveRole（主攻手）**：距离球最近，负责进攻
- **AssistRole（协助者）**：支援进攻，寻找传球机会
- **PassiveRole（防守者）**：防守位置，拦截对手
- **MidfieldRole（中场）**：中场控制，攻守转换
- **GoalieRole（守门员）**：守门专项策略

---

### 3. 角色执行系统（Role System）

#### 📄 `activerole.cpp` - 主攻手角色 [重点模块]
**职责：** 最核心的进攻角色，负责带球、射门、组织进攻

**状态机：**
```
[看不到球] → [追球] → [接近球] → [带球] → [射门准备] → [射门]
     ↓                                 ↓
[搜索球]                           [传球]
```

**核心功能：**
- ⚽ 追球与带球控制
- 🎯 射门角度计算与执行
- 🔄 传球时机判断
- 🚧 动态避障
- 📊 对手守门员位置预测

**代码亮点：**
```cpp
// 射门决策逻辑
bool ActiveRole::canShoot()
{
    // 计算射门角度
    double shoot_angle = calculateShootAngle();
    
    // 检查守门员位置
    if(isGoalieBlocking(shoot_angle))
        return false;
    
    // 检查射门距离
    double distance = robot_pos_.distance(goal_center);
    if(distance > MAX_SHOOT_DISTANCE)
        return false;
        
    return true;
}
```

#### 📄 `assistrole.cpp` - 协助角色
**职责：** 无球跑位，创造传球机会，支援进攻

**核心算法：**
- 计算最佳接应位置（考虑传球路径和对手位置）
- 视野分析：判断是否有清晰传球路线
- 空间创造：拉扯防守，制造空档

**位置计算：**
```cpp
// 计算最佳协助位置
DPoint AssistRole::calculateAssistPosition()
{
    DPoint ball_pos = getBallPosition();
    DPoint active_robot_pos = getActiveRobotPosition();
    
    // 在传球路径上寻找最佳位置
    // 避免越位和进入禁区
    // 保持合理距离
    
    return optimal_position;
}
```

#### 📄 `passiverole.cpp` - 防守角色
**职责：** 防守定位，拦截对手，保护球门

**防守策略：**
- 区域防守：根据威胁程度分配防守位置
- 预判拦截：预测球的运动轨迹进行拦截
- 回防协助：进攻转防守时快速回位

#### 📄 `midfieldrole.cpp` - 中场角色
**职责：** 中场控制，连接攻防，覆盖中场区域

**核心任务：**
- 中场覆盖：控制中场区域
- 攻守转换：快速攻防转换
- 支援两端：根据需要支援进攻或防守

#### 📄 `goaliestrategy.cpp` - 守门员策略
**职责：** 守门专项，最后一道防线

**守门技术：**
- 位置选择：根据球的位置调整守门位置
- 扑救动作：预判射门方向
- 大脚解围：危险情况下的解围

---

### 4. 规划控制系统（Planning & Control）

#### 📄 `plan.cpp` - 路径规划器
**功能：**
- 全向移动控制
- 动态避障算法
- 速度规划与限制
- 目标点导航

**避障策略：**
```cpp
void Plan::move2PositionWithObs(DPoint target)
{
    // 检测障碍物
    vector<Obstacle> obstacles = detectObstacles();
    
    // 计算避障路径
    DPoint waypoint = calculateAvoidPath(target, obstacles);
    
    // 平滑轨迹
    smoothPath(waypoint);
    
    // 执行运动
    executeMotion(waypoint);
}
```

#### 📄 `subtargets.cpp` - 子目标生成器
**功能：** 将复杂任务分解为可执行的子目标序列

#### 📄 `behaviour.cpp` - 行为控制器
**功能：**
- 运动基元实现
- 速度与加速度控制
- 动作序列执行

**运动控制：**
```cpp
// 全向移动控制
void Behaviour::omnidirectionalMove(DPoint target, double max_vel)
{
    DPoint error = target - current_pos;
    
    // 计算速度分量
    double vx = kp * error.x_;
    double vy = kp * error.y_;
    double w = kw * angle_error;
    
    // 速度限制
    limitVelocity(vx, vy, w, max_vel);
    
    // 发送速度指令
    publishVelocityCommand(vx, vy, w);
}
```

#### 📄 `bezier.cpp` - 贝塞尔曲线轨迹
**功能：** 生成平滑的运动轨迹，用于优雅的路径规划

---

### 5. 传球协作系统（Pass System）

#### 📄 `passstrategy.cpp` - 传球策略
**传球类型：**
- 静态传球：定位球传球
- 动态传球：运动中传球

**传球决策：**
```cpp
bool shouldPass()
{
    // 检查是否有队友在合适位置
    if(!hasTeammateInPosition())
        return false;
    
    // 检查传球路径是否畅通
    if(isPathBlocked())
        return false;
    
    // 检查自身是否受到威胁
    if(isUnderPressure())
        return true;
        
    return false;
}
```

#### 📄 `staticpass.cpp` - 静态传球实现
**功能：**
- 定位球站位计算
- 传球目标选择
- 接球位置预测
- 传球力度控制

---

### 6. 辅助系统（Utility）

#### 📄 `world_model_info.cpp` - 世界模型
**功能：**
- 全局信息维护
- 多机器人状态融合
- 球位置预测
- 对手信息估计

#### 📄 `fieldinformation.cpp` - 场地信息
**功能：**
- 场地边界检测
- 禁区判断
- 越位检测
- 位置合法性验证

#### 📄 `test.cpp` - 测试与调试
**功能：**
- 单元测试
- 策略验证
- 性能分析

---

## 💡 技术亮点

### 1. 分布式决策架构
- 每个机器人独立决策，通过ROS话题共享信息
- 避免单点故障，提高系统鲁棒性
- 实现了去中心化的角色分配算法

### 2. 多层次状态机
```
全局状态机（比赛阶段）
    ├── 策略状态机（战术选择）
    │   ├── 角色状态机（角色行为）
    │   │   └── 动作状态机（具体动作）
```

### 3. 实时性优化
- 15ms控制周期内完成全部计算
- 高效的数据结构设计
- 避免不必要的重复计算

### 4. 智能传球算法
- 考虑队友位置、对手分布、传球路径
- 动态评估传球成功率
- 支持多种传球战术

### 5. 精确运动控制
- 全向移动平台的精确控制
- 动态避障算法
- 平滑的轨迹规划（贝塞尔曲线）

---

## 📁 代码结构

```
simatch/
└── src/
    └── robot_code/
        └── nubot_control/
            └── src/                              # 策略实现代码（本项目核心）
                ├── main.cpp                      # 程序入口
                ├── nubot_control.cpp            # 主控制器（约900行）
                │
                ├── 策略决策模块
                │   ├── strategy.cpp             # 策略引擎
                │   ├── role_assignment.cpp      # 角色分配器
                │   └── test.cpp                 # 测试模块
                │
                ├── 角色执行模块
                │   ├── activerole.cpp           # 主攻手角色（约1000行）⭐
                │   ├── assistrole.cpp           # 协助角色（约400行）
                │   ├── passiverole.cpp          # 防守角色
                │   ├── midfieldrole.cpp         # 中场角色
                │   └── goaliestrategy.cpp       # 守门员策略
                │
                ├── 规划控制模块
                │   ├── plan.cpp                 # 路径规划器
                │   ├── subtargets.cpp           # 子目标生成
                │   ├── behaviour.cpp            # 行为控制
                │   └── bezier.cpp               # 贝塞尔曲线
                │
                ├── 传球协作模块
                │   ├── passstrategy.cpp         # 传球策略
                │   └── staticpass.cpp           # 静态传球
                │
                └── 辅助模块
                    ├── world_model_info.cpp     # 世界模型
                    └── fieldinformation.cpp     # 场地信息
```

**统计数据：**
- 📝 总文件数：18个核心模块
- 💻 代码量：约3000+行（不含注释）
- ⏱️ 开发周期：比赛备战期间
- 🐛 调试优化：经过多轮迭代和实战测试

---

## 🛠️ 运行环境

### 依赖平台
本策略代码需要运行在中国机器人大赛提供的官方仿真平台上。

### 环境要求
- **操作系统**：Ubuntu 16.04 LTS (推荐) / Ubuntu 14.04
- **ROS版本**：ROS Kinetic (推荐) / ROS Jade
- **仿真器**：Gazebo 7.0+ / Gazebo 5.0+
- **编译器**：GCC 5.4+ (支持C++11)
- **依赖库**：Boost, Eigen3

### 获取完整平台

```bash
# 1. 克隆官方仿真平台
git clone https://github.com/nubot-nudt/simatch.git
cd simatch/simatch

# 2. 替换策略代码
# 将本项目的src文件夹替换到以下位置：
# simatch/src/robot_code/nubot_control/src/

# 3. 编译
catkin_make

# 4. 配置环境
source devel/setup.bash
```

---

## 🚀 快速开始

### 基本运行

```bash
# 终端1：启动仿真环境
roslaunch nubot_gazebo game_ready.launch

# 终端2：运行策略代码（cyan队）
rosrun nubot_common cyan_robot.sh

# 终端3：启动比赛控制
rosrun auto_referee auto_referee -1
```

### 单机完整测试

```bash
# 一键启动（包含仿真环境和策略代码）
roslaunch simatch_cyan.launch
```

### 双方对抗测试

```bash
# 同时启动两队进行对抗
# 终端1：cyan队
rosrun nubot_common cyan_robot.sh

# 终端2：magenta队  
rosrun nubot_common magenta_robot.sh

# 终端3：自动裁判
rosrun auto_referee auto_referee -1
```

### 调试技巧

```bash
# 查看机器人发布的策略信息
rostopic echo /nubot1/nubotcontrol/strategy

# 查看运动控制指令
rostopic echo /nubot1/nubotcontrol/actioncmd

# 可视化节点关系
rosrun rqt_graph rqt_graph

# 录制比赛数据
rosbag record -a
```

---


### 技术难点突破

1. **15ms实时控制**
   - 问题：控制周期短，计算时间有限
   - 解决：优化算法复杂度，减少不必要计算

2. **多机器人协调**
   - 问题：多个机器人同时追球，产生冲突
   - 解决：设计距离优先的角色分配算法

3. **动态传球**
   - 问题：运动中的传球接球时机难以把握
   - 解决：实现位置预测算法，提前计算接球点

4. **对手建模**
   - 问题：不同对手策略差异大
   - 解决：设计自适应策略，根据对手行为调整


---

## 📚 相关资源

### 官方资源
- [比赛官方仿真平台](https://github.com/nubot-nudt/simatch)
- [RoboCup中型组规则](http://www.robocup.org/)
- [ROS官方文档](http://wiki.ros.org/)



---

