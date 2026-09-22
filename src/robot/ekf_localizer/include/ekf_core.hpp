#ifndef EKF_CORE_HPP_
#define EKF_CORE_HPP_

#include <array>

namespace robot
{

/**
 * @class EkfCore
 * @brief 扩展卡尔曼滤波核心（纯数学，无 ROS 依赖，可独立单测）
 *
 * 状态量 x = [x, y, θ]ᵀ（平面 3 自由度，2D 差分底盘）
 * 预测：差分底盘运动模型 + Thrun 过程噪声
 * 更新：轮速里程计位姿观测（3 维）+ IMU 角速度观测（1 维）
 */
class EkfCore {
  public:
    EkfCore();

    /**
     * @brief 重置滤波器到初始位姿
     */
    void reset(double x, double y, double theta);

    /**
     * @brief 预测步：差分底盘运动模型推状态，协方差经雅可比 F 传播并叠加过程噪声 Q
     * @param v 线速度控制量 (m/s，来自轮速里程计 twist)
     * @param omega 角速度控制量 (rad/s)
     * @param dt 步长 (s)
     */
    void predict(double v, double omega, double dt);

    /**
     * @brief 位姿观测更新：z = [x, y, θ]（来自带噪轮速里程计）
     * @param z 观测值数组
     * @param r 观测噪声方差（对角线 3 元素）
     * @return true 观测有效并完成更新
     */
    bool updatePose(const std::array<double, 3>& z, const std::array<double, 3>& r);

    /**
     * @brief IMU 角速度观测更新：只修正 θ 分量（z ≈ θ + ω·dt 的间接单量更新）
     * @param yaw_meas IMU 报告的偏航角（积分型处理，见实现）
     * @param r 观测噪声方差
     * @return true 观测有效并完成更新
     */
    bool updateYaw(double yaw_meas, double r);

    /**
     * @brief 设置 Thrun 运动模型过程噪声系数
     */
    void setMotionNoise(double a1, double a2, double a3, double a4) {
      alpha1_v_ = a1; alpha2_w_ = a2; alpha3_v_ = a3; alpha4_w_ = a4;
    }

    /// 当前状态估计
    double x() const { return x_[0]; }
    double y() const { return x_[1]; }
    double theta() const { return x_[2]; }

    /// 协方差对角元素（供发布 Odometry covariance）
    double covX() const { return P_[0][0]; }
    double covY() const { return P_[1][1]; }
    double covTheta() const { return P_[2][2]; }

  private:
    double x_[3];       ///< 状态 [x, y, θ]
    double P_[3][3];    ///< 3x3 协方差矩阵
    bool initialized_ = false;

    // Thrun 速度运动模型过程噪声系数（由节点层通过 setMotionNoise 设置）
    double alpha1_v_ = 0.10;  ///< 线速度噪声随速度比例项
    double alpha2_w_ = 0.01;  ///< 线速度噪声随角速度耦合项
    double alpha3_v_ = 0.01;  ///< 角速度噪声随速度耦合项
    double alpha4_w_ = 0.10;  ///< 角速度噪声随角速度比例项

    /// 角度归一化到 [-π, π]
    static double wrapAngle(double a);
};

}  // namespace robot

#endif  // EKF_CORE_HPP_
