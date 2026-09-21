# Copyright 2023 WATonomous
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from sample_msgs.msg import Unfiltered, Filtered, FilteredArray
from transformer.transformer_core import TransformerCore


class Transformer(Node):
    """ROS 2 Python 版转换器节点：订阅未滤波话题，解析坐标后批量打包发布至 /filtered_topic。"""

    def __init__(self):
        super().__init__('python_transformer')
        # 声明节点参数
        self.declare_parameter('version', 1)
        self.declare_parameter('compression_method', 0)
        self.declare_parameter('buffer_capacity', 5)

        self.__buffer_capacity = self.get_parameter('buffer_capacity') \
            .get_parameter_value().integer_value

        # 初始化转换器算法核心
        self.__transformer = TransformerCore()

        # 初始化 ROS 2 话题发布者与订阅者
        self.publisher_ = self.create_publisher(FilteredArray, '/filtered_topic', 10)
        self.subscription = self.create_subscription(Unfiltered, '/unfiltered_topic',
                                                     self.unfiltered_callback, 10)

        # 批处理消息缓存列表
        self.__filtered_array_packets = []

    def unfiltered_callback(self, msg):
        """未滤波原始消息回调处理。"""
        # 校验消息合法性
        if not self.check_msg_validity(msg):
            self.get_logger().info('收到无效消息 (INVALID MSG)')
            return

        # 创建并填充 Filtered 结构化消息对象
        filtered_msg = Filtered()
        filtered_msg.pos_x, filtered_msg.pos_y, filtered_msg.pos_z = self.__transformer \
            .deserialize_data(msg.data)
        filtered_msg.timestamp = msg.timestamp
        filtered_msg.metadata.version = self.get_parameter('version') \
            .get_parameter_value().integer_value
        filtered_msg.metadata.compression_method = self.get_parameter('compression_method') \
            .get_parameter_value().integer_value

        # 压入批处理数组队列，若未满容量则等待后续数据
        if self.populate_packet(filtered_msg):
            return

        # 达到容量上限，打包为 FilteredArray 并发布
        filtered_array_msg = FilteredArray()
        filtered_array_msg.packets = self.__filtered_array_packets

        self.get_logger().info('已达缓冲区容量上限，正在发布批量数据 (PUBLISHING)...')
        self.publisher_.publish(filtered_array_msg)

        # 清空当前批次以迎接下一组数据
        self.__filtered_array_packets.clear()

    def populate_packet(self, filtered_msg):
        """将消息加入列表并检查是否已满容量。"""
        self.__filtered_array_packets.append(filtered_msg)
        return len(self.__filtered_array_packets) <= self.__buffer_capacity

    def check_msg_validity(self, msg):
        """校验消息的 valid 标志位。"""
        return msg.valid


def main(args=None):
    """节点主入口函数。"""
    rclpy.init(args=args)

    python_transformer = Transformer()
    rclpy.spin(python_transformer)

    # 显式销毁节点并关闭客户端库
    python_transformer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

