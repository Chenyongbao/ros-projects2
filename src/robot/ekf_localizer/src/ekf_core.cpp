// ekf_core.cpp
// 扩展卡尔曼滤波核心实现：3-DOF 状态 (x, y, θ)，预测（运动模型）+ 更新（位姿/偏航观测）
// 纯数学实现，无 ROS 消息依赖，可独立编译与单测

#include "ekf_core.hpp"

#include <cmath>

namespace robot
{

EkfCore::EkfCore() {
  reset(0.0, 0.0, 0.0);
}

double EkfCore::wrapAngle(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

/**
 * @brief 重置滤波器到指定位姿，协方差取单位阵（不确定性最大）
 */
void EkfCore::reset(double x, double y, double theta) {
  x_[0] = x;
  x_[1] = y;
  x_[2] = theta;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      P_[i][j] = (i == j) ? 1.0 : 0.0;
  initialized_ = true;
}

/**
 * @brief 预测步
 * 状态：x' = x + v·cosθ·dt, y' = y + v·sinθ·dt, θ' = θ + ω·dt
 * 协方差：P⁻ = F·P·Fᵀ + Q
 *   F = [[1, 0, -v·sinθ·dt],
 *        [0, 1,  v·cosθ·dt],
 *        [0, 0,  1         ]]
 * 过程噪声：速度噪声映射到位置/角度增量（对角近似）
 */
void EkfCore::predict(double v, double omega, double dt) {
  if (!initialized_) return;

  const double th = x_[2];
  const double c = std::cos(th), s = std::sin(th);

  // 1. 状态推進
  x_[0] += v * c * dt;
  x_[1] += v * s * dt;
  x_[2] = wrapAngle(x_[2] + omega * dt);

  // 2. 雅可比 F
  const double F[3][3] = {
    {1.0, 0.0, -v * s * dt},
    {0.0, 1.0,  v * c * dt},
    {0.0, 0.0,  1.0       }
  };

  // 3. 过程噪声 Q（对角）：速度噪声经 dt 传播到位姿增量
  //    σ_pos = σ_v·dt, σ_yaw = σ_ω·dt（σ 由节点层按 Thrun 模型算好传入不便，此处用简化内联）
  //    为保持 core 纯数学，噪声强度以参数等效形式并入：Q 与 dt² 成正比
  const double q_pos = (alpha1_v_ * v * v + alpha2_w_ * omega * omega) * dt * dt;
  const double q_yaw = (alpha3_v_ * v * v + alpha4_w_ * omega * omega) * dt * dt;

  // 4. P⁻ = F·P·Fᵀ + Q
  double FP[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      FP[i][j] = 0.0;
      for (int k = 0; k < 3; ++k)
        FP[i][j] += F[i][k] * P_[k][j];
    }
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      double PFT = 0.0;
      for (int k = 0; k < 3; ++k)
        PFT += FP[i][k] * F[j][k];  // Fᵀ[k][j] = F[j][k]
      P_[i][j] = PFT;
    }
  P_[0][0] += q_pos;
  P_[1][1] += q_pos;
  P_[2][2] += q_yaw;
}

/**
 * @brief 位姿观测更新（z = [x, y, θ]，H = I₃）
 * K = P(HᵀHP+R)⁻¹Hᵀ 化简为标量逐维更新（对角 R）：
 *   K_i = P_ii / (P_ii + r_i)（保留 H=I 时对角近似，工程足够）
 * 实际按完整 3x3 公式实现：S = P + R（对角），K = P·S⁻¹
 */
bool EkfCore::updatePose(const std::array<double, 3>& z, const std::array<double, 3>& r) {
  if (!initialized_) return false;

  // S = P + R（H = I 时的新息协方差，R 取对角）
  double S[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      S[i][j] = P_[i][j] + ((i == j) ? r[i] : 0.0);

  // 求 S 的逆（3x3 直接法：先伴随矩阵再除行列式）
  double det =
      S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1])
    - S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0])
    + S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
  if (std::abs(det) < 1e-12) return false;
  const double inv = 1.0 / det;

  double Sinv[3][3];
  Sinv[0][0] =  (S[1][1] * S[2][2] - S[1][2] * S[2][1]) * inv;
  Sinv[0][1] = -(S[0][1] * S[2][2] - S[0][2] * S[2][1]) * inv;
  Sinv[0][2] =  (S[0][1] * S[1][2] - S[0][2] * S[1][1]) * inv;
  Sinv[1][0] = -(S[1][0] * S[2][2] - S[1][2] * S[2][0]) * inv;
  Sinv[1][1] =  (S[0][0] * S[2][2] - S[0][2] * S[2][0]) * inv;
  Sinv[1][2] = -(S[0][0] * S[1][2] - S[0][2] * S[1][0]) * inv;
  Sinv[2][0] =  (S[1][0] * S[2][1] - S[1][1] * S[2][0]) * inv;
  Sinv[2][1] = -(S[0][0] * S[2][1] - S[0][2] * S[2][0]) * inv;
  Sinv[2][2] =  (S[0][0] * S[1][1] - S[0][1] * S[1][0]) * inv;

  // K = P·S⁻¹（3x3 矩阵乘）
  double K[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      K[i][j] = 0.0;
      for (int k = 0; k < 3; ++k)
        K[i][j] += P_[i][k] * Sinv[k][j];
    }

  // 新息 y = z - x（角度维归一化）
  const double innov[3] = {
    z[0] - x_[0],
    z[1] - x_[1],
    wrapAngle(z[2] - x_[2])
  };

  // 状态更新：x' = x + K·y
  for (int i = 0; i < 3; ++i) {
    double dy = 0.0;
    for (int k = 0; k < 3; ++k) dy += K[i][k] * innov[k];
    x_[i] += dy;
  }
  x_[2] = wrapAngle(x_[2]);

  // 协方差更新：P = (I - K)·P
  double ImK[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      ImK[i][j] = ((i == j) ? 1.0 : 0.0) - K[i][j];

  double newP[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      newP[i][j] = 0.0;
      for (int k = 0; k < 3; ++k)
        newP[i][j] += ImK[i][k] * P_[k][j];
    }
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) P_[i][j] = newP[i][j];

  return true;
}

/**
 * @brief IMU 偏航观测更新：单量（θ 维）KF 更新
 * IMU 陀螺仪短时 yaw 精度高，用其对 θ 做高频纠偏
 */
bool EkfCore::updateYaw(double yaw_meas, double r) {
  if (!initialized_ || r <= 0.0) return false;

  const double innov = wrapAngle(yaw_meas - x_[2]);
  const double K = P_[2][2] / (P_[2][2] + r);  // 标量卡尔曼增益

  x_[2] = wrapAngle(x_[2] + K * innov);

  // P' = (1 - K)·P_θθ（θ 与 x/y 的耦合项同比例收缩）
  const double scale = 1.0 - K;
  for (int j = 0; j < 3; ++j) {
    P_[2][j] *= scale;
    P_[j][2] *= scale;
  }
  return true;
}

}  // namespace robot
