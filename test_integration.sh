#!/bin/bash
# 集成测试脚本：验证 transport 和 inspection 订单的完整状态流转

set -e

echo "=========================================="
echo "  ROS2 订单系统集成测试"
echo "=========================================="

# 加载 ROS2 环境
source /opt/ros/*/setup.bash
source /home/mrchen/learn-V2-submit_order/install/setup.bash

# 清理可能残留的节点
echo "[1/7] 清理残留节点..."
ros2 node list 2>/dev/null | grep -E "order_gateway|mock_navigation" | while read node; do
    ros2 node kill "$node" 2>/dev/null || true
done
sleep 1

# 启动节点（auto_dispatch=true）
echo "[2/7] 启动节点 (auto_dispatch=true)..."
ros2 launch learn_order_node order_gateway.launch.py auto_dispatch:=true use_mock_navigation:=true &
LAUNCH_PID=$!

# 等待节点就绪
echo "[3/7] 等待节点就绪..."
for i in {1..30}; do
    if ros2 service list 2>/dev/null | grep -q "/v2/submit_order"; then
        echo "  ✓ 节点已就绪"
        break
    fi
    if [ $i -eq 30 ]; then
        echo "  ✗ 节点启动超时"
        kill $LAUNCH_PID 2>/dev/null
        exit 1
    fi
    sleep 0.5
done

# 等待 Action Server 就绪
echo "[4/7] 等待 Action Server 就绪..."
for i in {1..30}; do
    if ros2 action list 2>/dev/null | grep -q "/navigate_sequence"; then
        echo "  ✓ Action Server 已就绪"
        break
    fi
    if [ $i -eq 30 ]; then
        echo "  ✗ Action Server 启动超时"
        kill $LAUNCH_PID 2>/dev/null
        exit 1
    fi
    sleep 0.5
done

sleep 1

# 提交 transport 订单
echo ""
echo "[5/7] 提交 transport 订单..."
echo "---"
ros2 service call /v2/submit_order learn_order_interfaces/srv/SubmitOrder \
    "{order_id: 'test_transport_001', order_type: 'transport', priority: 5, payload_json: '{\"frame_id\":\"map\",\"pickup_x\":1.0,\"pickup_y\":2.0,\"pickup_yaw\":0.0,\"dropoff_x\":5.0,\"dropoff_y\":3.0,\"dropoff_yaw\":1.57}', tags: ['integration_test']}" 2>&1

sleep 2

# 提交 inspection 订单
echo ""
echo "[6/7] 提交 inspection 订单..."
echo "---"
ros2 service call /v2/submit_order learn_order_interfaces/srv/SubmitOrder \
    "{order_id: 'test_inspection_001', order_type: 'inspection', priority: 3, payload_json: '{\"frame_id\":\"map\",\"x\":8.0,\"y\":4.0,\"yaw\":0.0}', tags: ['integration_test']}" 2>&1

sleep 2

# 监控状态流转
echo ""
echo "[7/7] 监控状态流转 (等待最多30秒)..."
echo "=========================================="
echo "  订阅 /mission_state_structured"
echo "=========================================="

# 使用 timeout 监控状态，收集所有状态事件
TIMEOUT=30
ros2 topic echo /mission_state_structured --once 2>/dev/null &
MONITOR_PID=$!

# 等待并收集多个状态更新
COLLECTED_STATES=""
for i in $(seq 1 $TIMEOUT); do
    # 读取主题消息
    STATE_MSG=$(timeout 1 ros2 topic echo /mission_state_structured --once 2>&1 || true)
    if [ -n "$STATE_MSG" ]; then
        echo "$STATE_MSG"
        # 提取状态
        STATE=$(echo "$STATE_MSG" | grep "state:" | head -1 | awk '{print $2}')
        MISSION_ID=$(echo "$STATE_MSG" | grep "mission_id:" | head -1 | awk '{print $2}')
        if [ -n "$STATE" ] && [ -n "$MISSION_ID" ]; then
            COLLECTED_STATES="${COLLECTED_STATES}${MISSION_ID}:${STATE},"
        fi
    fi

    # 检查是否两个订单都达到终态
    TRANSPORT_DONE=$(echo "$COLLECTED_STATES" | grep -o "test_transport_001:SUCCEEDED" | wc -l)
    INSPECTION_DONE=$(echo "$COLLECTED_STATES" | grep -o "test_inspection_001:SUCCEEDED" | wc -l)

    if [ "$TRANSPORT_DONE" -gt 0 ] && [ "$INSPECTION_DONE" -gt 0 ]; then
        echo ""
        echo "=========================================="
        echo "  ✓ 两个订单都已达到 SUCCEEDED 状态"
        echo "=========================================="
        break
    fi
done

# 查询最终状态
echo ""
echo "=========================================="
echo "  最终状态查询"
echo "=========================================="

echo "--- test_transport_001 ---"
ros2 service call /mission_status learn_order_interfaces/srv/GetMissionStatus \
    "{mission_id: 'test_transport_001'}" 2>&1

echo ""
echo "--- test_inspection_001 ---"
ros2 service call /mission_status learn_order_interfaces/srv/GetMissionStatus \
    "{mission_id: 'test_inspection_001'}" 2>&1

# 查询事件历史
echo ""
echo "=========================================="
echo "  事件历史"
echo "=========================================="
ros2 service call /mission_events learn_order_interfaces/srv/GetMissionEvents \
    "{mission_id: 'test_transport_001'}" 2>&1

echo ""
ros2 service call /mission_events learn_order_interfaces/srv/GetMissionEvents \
    "{mission_id: 'test_inspection_001'}" 2>&1

# 清理
echo ""
echo "=========================================="
echo "  清理节点"
echo "=========================================="
kill $LAUNCH_PID 2>/dev/null || true
sleep 1
echo "测试完成!"
