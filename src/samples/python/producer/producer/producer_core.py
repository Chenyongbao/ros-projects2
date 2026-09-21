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

import math


class ProducerCore():
    """生产者核心逻辑类，负责在三维空间中按指定速度累加坐标并序列化为字符串。"""

    def __init__(self, pos_x, pos_y, pos_z, vel):
        """初始化空间三维坐标与移动速度。

        Args:
            pos_x (float): 初始 X 坐标
            pos_y (float): 初始 Y 坐标
            pos_z (float): 初始 Z 坐标
            vel (float): 移动速度标量
        """
        # 初始化私有成员变量
        self.__pos_x = pos_x
        self.__pos_y = pos_y
        self.__pos_z = pos_z
        self.__velocity = vel

    def update_position(self):
        """沿空间对角线方向均匀递增三维坐标值 (各轴增量 = velocity / sqrt(3))。"""
        self.__pos_x += self.__velocity / math.sqrt(3)
        self.__pos_y += self.__velocity / math.sqrt(3)
        self.__pos_z += self.__velocity / math.sqrt(3)

    def serialize_data(self):
        """将当前坐标序列化为格式化字符串，如 'x:..;y:..;z:..;'。

        Returns:
            str: 格式化后的坐标字符串
        """
        return "x:" + str(self.__pos_x) + ";y:" + \
            str(self.__pos_y) + ";z:" + str(self.__pos_z) + ";"

