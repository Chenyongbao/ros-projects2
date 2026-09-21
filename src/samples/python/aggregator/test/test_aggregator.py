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

import time

from aggregator.aggregator_core import AggregatorCore


def test_calc_freq():
    """测试给定消息条数时的平均频率计算逻辑。"""
    start_time = time.time()
    aggregator_node = AggregatorCore(start_time)

    test_count = 5
    test_freq = test_count / (time.time() - start_time)

    # 验证计算出的频率合理性
    assert aggregator_node.calc_freq(5) - test_freq < 0


def test_freq_update():
    """测试未滤波与滤波频率增量更新与计算的合理性。"""
    start_time = time.time()
    aggregator_node = AggregatorCore(start_time)

    aggregator_node.update_filtered_freq()
    test_freq_filtered = 1 / (time.time() - start_time)

    aggregator_node.update_raw_freq()
    test_freq_raw = 1 / (time.time() - start_time)

    # 验证频率更新后大于 0 且有效
    assert aggregator_node.filtered_freq - test_freq_filtered > 0
    assert aggregator_node.raw_freq - test_freq_raw > 0

