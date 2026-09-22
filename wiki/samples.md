# 示例程序（samples）

> 源码：`src/samples/`，编排：`modules/docker-compose.samples.yaml`

`src/samples` 是一套独立于导航栈的**发布-订阅**（音标 /ˈpʌblɪʃ ˈsʌbskraɪb/：Pub/Sub 发布-订阅模式：解耦的消息通信范式，发布者不关心谁消费，是 ROS 2 通信的底层模型）教学示例，演示 ROS 2 的 C++ 与 Python 双语言开发范式。

## 结构

每个语言版本包含三个节点，构成一条数据流水线：

```text
producer ──发布──▶ 话题 ──▶ transformer ──▶ 话题 ──▶ aggregator ──▶ 结果话题
```

| 节点 | 角色 | 教学点 |
|---|---|---|
| **producer** | 周期发布自定义消息 | publisher、定时器、参数 |
| **transformer** | 订阅并变换数据再发布 | subscription 回调、消息再发布 |
| **aggregator** | 汇总统计并输出 | 状态累积、结果输出 |

## 自定义消息

`src/samples/sample_msgs/msg/` 定义了 `.msg` 接口文件，通过 `rosidl_generate_interfaces` 在构建期生成 C++/Python 绑定 —— 这是 ROS 2 接口包的标准写法（可对照 `src/wato_msgs/sample_msgs`）。

## 与导航栈的关系

samples 与 `src/robot` 完全独立，可作为热身练习：先跑通 producer→transformer→aggregator 链路，理解话题通信后再进入导航模块源码。
