#ifndef PRODUCER_CORE_HPP_
#define PRODUCER_CORE_HPP_

#include "sample_msgs/msg/unfiltered.hpp"

namespace samples
{

/**
 * @class ProducerCore
 * @brief 生产者内部算法实现类，负责三维坐标更新与字符串序列化
 */
class ProducerCore
{
public:
  /**
   * @brief 构造函数，初始化三维坐标 (x, y, z)
   * @param x 初始 X 坐标
   * @param y 初始 Y 坐标
   * @param z 初始 Z 坐标
   */
  explicit ProducerCore(float x = 0, float y = 0, float z = 0);

  /**
   * @brief 更新运动速度（由参数动态更新回调调用）
   * @param velocity 新的速度值
   */
  void update_velocity(int velocity);

  /**
   * @brief 更新初始位置坐标（静态参数）
   * @param pos_x X 初始位置
   * @param pos_y Y 初始位置
   * @param pos_z Z 初始位置
   */
  void update_position(double pos_x, double pos_y, double pos_z);

  /**
   * @brief 根据速度值更新三维坐标位置（各轴分量增量为 velocity / sqrt(3)）
   */
  void update_coordinates();

  /**
   * @brief 将当前三维坐标序列化为格式化字符串，如 "x:num1;y:num2;z:num3;"
   * @param[out] msg 待填充的未滤波消息对象
   */
  void serialize_coordinates(sample_msgs::msg::Unfiltered & msg) const;

private:
  // 空间三维坐标值
  double pos_x_;
  double pos_y_;
  double pos_z_;

  // 用于在固定时间间隔内累加坐标的速度参数
  double velocity_;
};

}  // namespace samples

#endif  // PRODUCER_CORE_HPP_

