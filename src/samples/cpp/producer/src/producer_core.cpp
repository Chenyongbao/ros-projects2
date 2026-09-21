// producer_core.cpp
// 生产者核心算法实现：处理坐标累加计算与格式化字符串封装

#include <chrono>
#include <cmath>

#include "producer_core.hpp"

namespace samples
{

/**
 * @brief 构造函数，初始化位置坐标与速度为 0
 */
ProducerCore::ProducerCore(float x, float y, float z)
: pos_x_(x), pos_y_(y), pos_z_(z), velocity_(0)
{
}

/**
 * @brief 更新当前运动速度
 */
void ProducerCore::update_velocity(int velocity)
{
  velocity_ = velocity;
}

/**
 * @brief 更新初始三维坐标
 */
void ProducerCore::update_position(double pos_x, double pos_y, double pos_z)
{
  pos_x_ = pos_x;
  pos_y_ = pos_y;
  pos_z_ = pos_z;
}

/**
 * @brief 坐标位置累加：沿空间三维对角线方向均匀增加（增量 = velocity / sqrt(3)）
 */
void ProducerCore::update_coordinates()
{
  pos_x_ += velocity_ / sqrt(3);
  pos_y_ += velocity_ / sqrt(3);
  pos_z_ += velocity_ / sqrt(3);
}

/**
 * @brief 将三维坐标数值拼装为未滤波的字符串格式： "x:..;y:..;z:..;"
 */
void ProducerCore::serialize_coordinates(sample_msgs::msg::Unfiltered & msg) const
{
  msg.data = "x:" + std::to_string(pos_x_) + ";y:" + std::to_string(pos_y_) +
    ";z:" + std::to_string(pos_z_) + ";";
  msg.valid = true;
}

}  // namespace samples

