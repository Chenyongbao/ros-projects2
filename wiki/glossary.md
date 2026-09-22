# 术语表

本页汇总 wiki 中出现的全部专业名词。**正文中所有带虚线下划线的词，鼠标移入即可查看音标与解释**。

## ROS 2 / 中间件

| 术语 | 音标 | 解释 |
|---|---|---|
| Node | /nəʊd/ | ROS 2 中最小的可执行计算单元，通过话题/服务与其他节点通信 |
| Topic | /ˈtɒpɪk/ | 异步发布-订阅通信通道，无缓冲阻塞发送方 |
| Message | /ˈmɛsɪdʒ/ | 话题上传输的结构化数据类型，由 `.msg` 文件定义 |
| LaserScan | /ˈleɪzə skæn/ | 单帧 2D 激光扫描消息：ranges 数组 + 角度范围 + 量程 |
| OccupancyGrid | /ˈɒkjəpənsi ˈɡrɛdɪk/ | 占据栅格地图消息：resolution/width/height/origin + int8 代价数组（-1 未知，0 空闲，100 占据） |
| Odometry | /ɒˈdɒmɪtri/ | 里程计消息：机器人位姿（位置 + 四元数朝向）与速度 |
| Twist | /twɪst/ | 速度指令消息：linear（线速度 xyz）+ angular（角速度 xyz） |
| TF | /tiː ɛf/ | ROS 的坐标变换树，管理多坐标系间的实时变换关系 |

## 算法 / 数学

| 术语 | 音标 | 解释 |
|---|---|---|
| A\* | /eɪ stɑːr/ | 启发式图搜索算法，f(n)=g(n)+h(n)，启发可采纳时保证最优路径 |
| Heuristic | /hjʊəˈrɪstɪk/ | 启发函数：对剩余代价的乐观估计；欧氏距离对 8 邻域可采纳 |
| Inflation | /ɪnˈfleɪʃn/ | 障碍物膨胀：按距离衰减在障碍周围叠加代价，形成安全缓冲 |
| Rigid Transform | /ˈrɪdɪd ˈtrænsfɔːm/ | 刚体变换：仅含旋转 + 平移，保持形状与距离不变 |
| Quaternion | /ˈkwɔːtəniːn/ | 四元数 (x,y,z,w)：无万向锁的三维旋转表示 |
| Yaw | /jɔː/ | 偏航角：绕竖直 Z 轴的旋转角，即地面机器人的"车头朝向" |
| Pure Pursuit | /pjʊə ˈpɜːsjuːt/ | 纯追踪：追踪路径上前瞻点的几何路径跟踪算法 |
| Lookahead Point | /ˈlʊkəhɑːd pɔɪnt/ | 前瞻点：Pure Pursuit 追踪的目标路径点，决定跟踪平滑度 |
| P Control | /ˈprəʊpɔːʃənl kənˈtrəʊl/ | 比例控制：输出 ∝ 误差的经典反馈控制律 |
| Euclidean Distance | /ɪˈklɪdiən ˈdɪstəns/ | 欧氏距离：两点直线距离 √(dx²+dy²) |

## 机器人 / 系统

| 术语 | 音标 | 解释 |
|---|---|---|
| Differential Drive | /ˈdɪfrənʃl ˈdraɪvə/ | 差分驱动：左右轮独立调速实现转向的底盘结构 |
| Ego-centric | /ˈɛɡəʊ ˈsɛntrɪk/ | 以自我为中心（的坐标系）：原点固定在车体上随车移动 |
| Frame | /freɪm/ | 坐标系：TF 变换树中的参考系节点，如 `sim_world`、`base_link` |
| Ground Truth | /ɡraʊnd truːθ/ | 真值：仿真器或测量提供的无误差基准数据 |
| Docker | /ˈdɒkə/ | 容器化平台，保证开发环境可复现、免装依赖 |
| Foxglove | /ˈfɒksɡlʌv/ | 机器人可视化调试平台，通过 WebSocket 连接 ROS 2 并可视化/交互 |
