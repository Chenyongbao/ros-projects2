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

from transformer.transformer_core import TransformerCore


def test_deserialization():
    """测试三维坐标字符串的反序列化解析功能。"""
    transformer_core = TransformerCore()

    serialized_msg = "x:1;y:1;z:1;"
    x, y, z = transformer_core.deserialize_data(serialized_msg)

    # 断言解析出的三个坐标均为 1
    assert x == 1 and y == 1 and z == 1

