# Auto Robot Nav Wiki

一个基于 **C++ / ROS 2** 的端到端自主导航栈，面向带 2D 激光雷达的**差分驱动**（音标 /ˈdɪfrənʃl ˈdraɪvə/：通过左右轮独立控制速度实现转向与前进的底盘结构，无独立转向轮）仿真小车。小车接收用户在地图上点击的目标点后，自主规划路径、避开障碍物并行驶到目标位置，同时"记住"已经探索过的区域。

本项目为 [WATonomous ASD Admissions Assignment](https://wiki.watonomous.ca/) 的实现。

## 系统能一览

| 节点 | 角色 | 核心技术 |
|---|---|---|
| **Costmap** | 感知 | **极坐标**（音标 /pəʊˈlɑːkəʊd/：Polar coordinates：用 (距离, 角度) 描述点位置的坐标系）障碍物膨胀|—|Inflation：在障碍物周围叠加随距离衰减的代价值，形成安全缓冲区| |
| **Map Memory** | 世界模型 | 基于 2D **刚体变换**（音标 /ˈrɪdɪd ˈtrænsfɔːm/：Rigid transform：只包含旋转与平移、不改变形状的坐标变换）的持久化全局地图 |
| **Planner** | 全局动作 | 占据栅格上的 **A\*** 路径搜索 |
| **Control** | 局部动作 | **Pure Pursuit**（音标 /pjʊə ˈpɜːsjuːt/：Pure Pursuit 纯追踪：通过追踪路径上的前瞻点计算转向角的几何路径跟踪算法） + 原地旋转对齐 |

## 设计原则

每个模块都拆分为两层：

- `*_node` —— ROS 2 面向层：负责订阅/发布话题、参数加载
- `*_core` —— 纯算法层：不依赖 ROS，便于单元测试与复用

## 快速开始

前置条件：Docker + Docker Compose。

```bash
git clone https://github.com/emlyqi/auto-robot-nav.git
cd auto-robot-nav
./watod up
```

打开 Foxglove Studio，连接 `ws://localhost:9000`，在 3D 地图上点击目标点即可看到小车规划路径并自主行驶。

## 本 Wiki 如何阅读

- **[系统架构](./architecture)** —— 数据流、话题拓扑、坐标系约定
- **[模块详解](./costmap)** —— 四大核心模块逐行解读
- **[部署运行](./deploy)** —— Docker / watod 使用
- **[术语表](./glossary)** —— 全部专业名词索引

::: tip 阅读提示
文中所有虚线下划线标记的单词或技术名词，**鼠标移入即可看到音标与解释**，无需跳转查询。
:::
