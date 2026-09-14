#!/bin/bash
# 冒烟测试：提交订单 -> 自动派发 -> 查状态
source /opt/ros/humble/setup.bash
source /mnt/c/Users/Mr.chen/Desktop/ROS2/learn-V2-submit_order/install_verify/setup.bash

ros2 run learn_order_node order_gateway_node --ros-args -p auto_dispatch:=true >/tmp/gw.log 2>&1 &
GW=$!
ros2 run learn_order_node mock_navigation_server >/tmp/mock.log 2>&1 &
MOCK=$!
sleep 6

echo "=== submit ==="
ros2 service call /v2/submit_order learn_order_interfaces/srv/SubmitOrder \
  "{order_id: 'smoke_001', order_type: 'transport', priority: 10, \
    payload_json: '{\"frame_id\":\"map\",\"pickup_x\":1.0,\"pickup_y\":2.0,\"pickup_yaw\":0.0,\"dropoff_x\":5.0,\"dropoff_y\":3.0,\"dropoff_yaw\":1.57}', \
    tags: ['smoke']}" 2>&1 | grep -oE "accepted=[a-z]+, message='[^']*'"

sleep 4
echo "=== status ==="
ros2 service call /mission_status learn_order_interfaces/srv/GetMissionStatus \
  "{mission_id: 'smoke_001'}" 2>&1 | grep -oE "found=[A-Za-z]+, state='[^']*', is_terminal=[A-Za-z]+, retry_count=[0-9]+"

echo "=== events (tail) ==="
ros2 service call /mission_events learn_order_interfaces/srv/GetMissionEvents \
  "{mission_id: 'smoke_001'}" 2>&1 | grep -oE "event='[^']*'" | head -8

kill $GW $MOCK 2>/dev/null
