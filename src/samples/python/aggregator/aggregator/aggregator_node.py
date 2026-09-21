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
import time
from rclpy.node import Node

from sample_msgs.msg import Unfiltered, FilteredArray
from aggregator.aggregator_core import AggregatorCore


class Aggregator(Node):
    """ROS 2 Python 版聚合器节点：订阅未滤波与滤波话题，实时输出各话题的消息总数与传输频率。"""

    def __init__(self):
        super().__init__('python_aggregator')

        # 记录节点启动时刻并初始化核心逻辑对象
        node_start_time = time.time()
        self.__aggregator = AggregatorCore(node_start_time)

        # 创建两个话题的订阅者
        self.unfiltered_subcriber = self.create_subscription(Unfiltered,
                                                             '/unfiltered_topic',
                                                             self.unfiltered_callback,
                                                             10)
        self.filtered_subscriber = self.create_subscription(FilteredArray,
                                                            '/filtered_topic',
                                                            self.filtered_callback,
                                                            10)

    def unfiltered_callback(self, msg):
        """未滤波话题消息接收回调。"""
        self.__aggregator.update_raw_freq()
        self.print_freqs()

    def filtered_callback(self, msg):
        """滤波话题数组接收回调。"""
        self.__aggregator.update_filtered_freq()
        self.print_freqs()

    def print_freqs(self):
        """在终端打印当前话题接收计数与平均传输频率。"""
        self.get_logger().info('未滤波消息总数: ' +
                               str(self.__aggregator.num_unfiltered_msgs))
        self.get_logger().info('滤波消息总数: ' +
                               str(self.__aggregator.num_filtered_msgs))

        self.get_logger().info('生产者发送频率 (条/秒): ' + str(self.__aggregator.raw_freq))
        self.get_logger().info('转换器发送频率 (条/秒): ' + str(self.__aggregator.filtered_freq))


def main(args=None):
    """节点入口主函数。"""
    rclpy.init(args=args)

    python_aggregator = Aggregator()
    rclpy.spin(python_aggregator)

    # 显式销毁节点并关闭 ROS 2 客户端
    python_aggregator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

