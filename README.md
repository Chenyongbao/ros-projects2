# learn-V2-submit_order

第一阶段：统一坐标订单接入与任务队列。

```text
SubmitOrder Service
  -> order_router
  -> MissionProfile
  -> preflight
  -> priority queue
  -> dispatch_next Service
  -> /navigate_sequence Action
  -> mock navigation execution
  -> mission_state Topic
```

核心任务链只支持 `transport` / `transport_order`，连接 Mock 导航 Action，不连接 Nav2、Gazebo、VDA5050 或真实硬件。
REST 网关作为独立的可选接入层提供，不改变核心任务链路。

## 构建

在 ROS 2 Jazzy 工作空间中：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 运行

```bash
ros2 launch learn_order_node order_gateway.launch.py
```

可通过 launch 参数设置最大重试次数：

```bash
ros2 launch learn_order_node order_gateway.launch.py max_retries:=1
```

提交坐标订单：

```bash
ros2 service call /v2/submit_order \
  learn_order_interfaces/srv/SubmitOrder \
  "{order_id: 'order_001', order_type: 'transport', priority: 10, payload_json: '{\"frame_id\":\"map\",\"pickup_x\":1.0,\"pickup_y\":2.0,\"pickup_yaw\":0.0,\"dropoff_x\":5.0,\"dropoff_y\":3.0,\"dropoff_yaw\":1.57}', tags: ['cli']}"
```

查看状态并派发：

```bash
ros2 topic echo /mission_state
ros2 service call /dispatch_next std_srvs/srv/Trigger {}
ros2 topic echo /mission_event
```

同时发布结构化 Topic，正式业务代码应优先使用它们：

```bash
ros2 topic echo /mission_state_structured
ros2 topic echo /mission_event_structured
```

`/mission_state` 和 `/mission_event` 仍保留为字符串教学 Topic，便于对比“字符串协议”和“结构化接口”的区别。

查询事件历史。传入任务 ID 查询单个任务；传入空字符串查询全部事件：

```bash
ros2 service call /mission_events \
  learn_order_interfaces/srv/GetMissionEvents \
  "{mission_id: 'order_001'}"
```

派发后可以看到 `/navigate_sequence` 的 feedback，以及 `RUNNING`、`SUCCEEDED`、`CANCELED` 等状态。

任务进入 `SUCCEEDED` / `FAILED` / `CANCELED` 任一终态后，同一 `mission_id` 允许重新提交订单；
排队中和执行中的同 ID 订单仍会被 `duplicate` 拦截。

取消任务（`CancelMission.srv`，带 `mission_id`）：

```bash
# 取消排队中的任务：直接移出队列并记录 CANCELED 终态
ros2 service call /cancel_mission learn_order_interfaces/srv/CancelMission "{mission_id: 'order_002'}"

# 取消正在执行的任务：向 Action Server 发起 Goal 取消，最终 CANCELED 由 Action 结果确认
ros2 service call /cancel_mission learn_order_interfaces/srv/CancelMission "{mission_id: 'order_001'}"

# 空 mission_id 保留旧行为：取消当前活动任务
ros2 service call /cancel_mission learn_order_interfaces/srv/CancelMission "{mission_id: ''}"
```

查询单个任务的最新状态（`GetMissionStatus.srv`）：

```bash
ros2 service call /mission_status learn_order_interfaces/srv/GetMissionStatus "{mission_id: 'order_001'}"
```

响应包含 `found`、`state`、`is_terminal`（是否已进入终态）、`retry_count` 与终态时刻 `timestamp`。

默认需要手动调用 `/dispatch_next`。如果希望节点自动从队列取任务，启动时打开定时调度：

```bash
ros2 launch learn_order_node order_gateway.launch.py auto_dispatch:=true
```

## REST 网关

REST 网关是独立进程，只把 HTTP JSON 转换为 `/v2/submit_order` Service 调用。
先启动任务中枢，再启动网关：

```bash
ros2 launch learn_order_node order_gateway.launch.py auto_dispatch:=true
ros2 run learn_order_node rest_api_gateway.py --port 18080
```

健康检查：

```bash
curl http://127.0.0.1:18080/health
```

提交订单：

```bash
curl -X POST http://127.0.0.1:18080/v2/submit_order \
  -H 'Content-Type: application/json' \
  -H 'Idempotency-Key: rest-order-001' \
  -d '{
    "order_id": "order_rest_001",
    "order_type": "transport",
    "priority": 10,
    "payload": {
      "frame_id": "map",
      "pickup_x": 1.0,
      "pickup_y": 2.0,
      "pickup_yaw": 0.0,
      "dropoff_x": 5.0,
      "dropoff_y": 3.0,
      "dropoff_yaw": 1.57
    },
    "tags": ["rest"]
  }'
```

REST 也可以调用当前闭环的控制和查询接口：

```bash
# 手动派发队列中的下一个任务（auto_dispatch 未开启时使用）
curl -X POST http://127.0.0.1:18080/dispatch_next

# 按 mission_id 取消（排队任务直接落终态；在跑任务发起 Goal 取消）
curl -X POST http://127.0.0.1:18080/cancel_mission \
  -H 'Content-Type: application/json' \
  -d '{"mission_id": "order_rest_001"}'

# 不带 body 时保留旧行为：取消当前活动任务
curl -X POST http://127.0.0.1:18080/cancel_mission

# 查询单个任务的最新状态与是否终态
curl 'http://127.0.0.1:18080/mission_status?mission_id=order_rest_001'

# 查询单个任务的事件历史
curl 'http://127.0.0.1:18080/mission_events?mission_id=order_rest_001'
```

REST 层不直接操作队列，也不直接调用导航 Action；订单仍由 C++ 任务中枢统一路由、预检和调度。
同一个 `Idempotency-Key` 重试会重放第一次响应，不会再次调用提交 Service；如果同一个键对应不同订单内容，网关返回 `409`。

测试失败重试时：

```bash
ros2 launch learn_order_node order_gateway.launch.py fail_first_goal:=true max_retries:=1
```

第一次 Action 会失败，网关会记录 `RETRY_QUEUED`，然后自动再次派发；超过 `max_retries` 后进入 `FAILED` 终态。

## 目录说明

- `learn_order_interfaces`：ROS2 Service 和 Action 契约。
- `learn_order_core`：订单路由、任务模型、前置检查和队列；不依赖 `rclcpp`。
- `learn_order_node`：ROS2 适配层，只负责 Service、Publisher 和调用 core。

## 当前刻意没有实现的内容

当前默认使用 `NavigateSequence` Action 和 Mock Action Server。已经提供最小 Nav2 适配器，可在具备 Nav2 的环境中切换：

```bash
ros2 launch learn_order_node order_gateway.launch.py \
  use_mock_navigation:=false auto_dispatch:=true
```

Nav2 适配器会逐个把 `NavigateSequence` 的 waypoint 转成 `NavigateToPose` Goal，并把取消请求继续传递给 Nav2。它不负责建图、定位或底盘控制，这些仍由 Nav2 和机器人系统提供。
