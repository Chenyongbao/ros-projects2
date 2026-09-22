# 部署运行（Docker / watod）

## 前置条件

- Linux Ubuntu ≥ 22.04（或 Windows WSL / macOS）
- [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) + Docker Compose

> 为什么用 Docker？无需在裸机上安装任何开发库，保证环境可复现（**reproducibility**（音标 /ˌriːprəˌdjuːsəˈbɪləti/：Reproducibility 可复现性：任何人在任何机器上用相同配置得到一致结果，工程协作的基石））。

## 快速启动

```bash
git clone https://github.com/emlyqi/auto-robot-nav.git
cd auto-robot-nav
./watod up
```

`watod`（WATonomous docker 简写）是仓库根目录的编排脚本，封装 docker-compose 命令组：

| 常用命令 | 作用 |
|---|---|
| `./watod up` | 启动全部服务（仿真 + 机器人节点） |
| `./watod build` | 构建各容器镜像 |
| `./watod down` | 停止并移除容器 |
| `./watod shell` | 进入某容器 shell 调试 |

## Compose 服务布局

| Compose 文件 | 服务 |
|---|---|
| `modules/docker-compose.gazebo.yaml` | Gazebo 仿真器（`docker/gazebo/gazeboserver.Dockerfile`） |
| `modules/docker-compose.robot.yaml` | 机器人节点容器（`docker/robot/robot.Dockerfile`），内含 4 个导航节点 |
| `modules/docker-compose.samples.yaml` | 示例程序（cpp / py producer-transformer-aggregator） |
| `modules/docker-compose.vis_tools.yaml` | Foxglove 可视化桥接 |

## 连接 Foxglove

1. 下载 [Foxglove Studio](https://foxglove.dev/)
2. Open Connection → Foxglove WebSocket → `ws://localhost:9000`
3. 在 3D 面板点击地图上的目标点（发布 `/clicked_point`）
4. 观察小车规划路径并行驶

`config/wato_asd_training_foxglove_config .json` 提供了预配置的面板布局。

## 常见问题

- **点击无反应**：确认 Planner 已收到 `/global_map`（Map Memory 需要小车移动过才有数据）
- **小车不动**：检查 `/plan` 是否为空（A* 不可达时返回空路径）
- **端口冲突**：9000 被占用时修改 `docker-compose.vis_tools.yaml` 的端口映射
