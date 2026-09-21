#ifndef PATH_SMOOTHER_HPP_
#define PATH_SMOOTHER_HPP_

#include <functional>
#include <vector>

namespace robot
{

/**
 * @brief 世界坐标系下的二维路径点
 */
struct PathPoint2D {
  double x;
  double y;
};

using PointPath2D = std::vector<PathPoint2D>;

/**
 * @brief 平滑结果：输出路径 + 是否触发了碰撞回退（平滑失败保留原路径）
 */
struct SmoothingResult2D {
  PointPath2D path;
  bool used_collision_fallback = false;
};

/**
 * @class PathResampler
 * @brief 路径重采样器：把任意间距的路径按固定弧长间距重新插值，使后续样条平滑的参数化均匀稳定
 */
class PathResampler {
  public:
    /**
     * @brief 按固定弧长间距对路径重采样
     * @param input_path 输入路径（世界坐标）
     * @param sample_spacing 采样间距（米）
     */
    PointPath2D resampleAtFixedSpacing(const PointPath2D& input_path, double sample_spacing) const;

    /**
     * @brief 计算路径总弧长（米）
     */
    double computePathLength(const PointPath2D& path) const;
};

/**
 * @class PathShortcutter
 * @brief 路径快捷化：基于视线检测贪心裁剪 A* 锯齿拐角，保留两端点不变
 */
class PathShortcutter {
  public:
    /**
     * @brief 视线快捷化
     * @param raw_path 原始路径
     * @param has_line_of_sight 视线检测回调：两点连线是否均处于自由空间
     */
    PointPath2D shortcut(
      const PointPath2D& raw_path,
      const std::function<bool(const PathPoint2D&, const PathPoint2D&)>& has_line_of_sight) const;
};

/**
 * @class SplinePathSmoother
 * @brief 自然三次样条平滑器：以弧长为参数，对 x(s)/y(s) 分别拟合 C² 连续样条并按固定间距重采样
 *        平滑后逐点做碰撞检测，失败则回退原始路径
 */
class SplinePathSmoother {
  public:
    /**
     * @brief 样条平滑
     * @param base_path 基准路径（已快捷化 + 重采样）
     * @param sample_spacing 输出路径采样间距（米）
     * @param map_resolution 地图分辨率，用于限制最小采样间距
     * @param is_point_in_free_space 碰撞检测回调：点是否处于自由空间
     */
    SmoothingResult2D smooth(
      const PointPath2D& base_path,
      double sample_spacing,
      double map_resolution,
      const std::function<bool(const PathPoint2D&)>& is_point_in_free_space) const;

  private:
    /// 弧长参数化：生成每个路径点的累计弧长 s
    std::vector<double> buildArcLengthParameter(const PointPath2D& points) const;

    /// 提取路径点的 x / y 序列
    std::vector<double> extractXPathValues(const PointPath2D& points) const;
    std::vector<double> extractYPathValues(const PointPath2D& points) const;

    /// 追赶法求解自然三次样条的二阶导数序列（三对角线性方程组）
    std::vector<double> solveNaturalCubicSecondDerivatives(
      const std::vector<double>& parameter_values,
      const std::vector<double>& sample_values) const;

    /// 按样条求值：给定弧长 s 返回插值坐标
    double evaluateNaturalCubicSpline(
      const std::vector<double>& parameter_values,
      const std::vector<double>& sample_values,
      const std::vector<double>& second_derivatives,
      double query_value) const;
};

}  // namespace robot

#endif  // PATH_SMOOTHER_HPP_
