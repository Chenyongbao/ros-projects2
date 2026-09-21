// path_smoother.cpp
// 路径后处理三件套实现：重采样 + 视线快捷化 + 自然三次样条平滑（参考 taorobot 方案）

#include "path_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace robot
{

// ==================== PathResampler ====================

/**
 * @brief 计算路径总弧长（米）
 */
double PathResampler::computePathLength(const PointPath2D& path) const {
  if (path.size() < 2) return 0.0;
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    double dx = path[i + 1].x - path[i].x;
    double dy = path[i + 1].y - path[i].y;
    total += std::sqrt(dx * dx + dy * dy);
  }
  return total;
}

/**
 * @brief 按固定弧长间距对路径重采样
 * 逐段线性插值，保证首尾点精确保留；过近的尾点去重
 */
PointPath2D PathResampler::resampleAtFixedSpacing(
  const PointPath2D& input_path, double sample_spacing) const
{
  if (input_path.size() < 2) return input_path;

  const double spacing = std::max(sample_spacing, 1e-3);
  PointPath2D out;
  out.reserve(
    std::max<std::size_t>(
      input_path.size(),
      static_cast<std::size_t>(std::ceil(computePathLength(input_path) / spacing)) + 1U));

  out.push_back(input_path.front());
  double dist_to_next_sample = spacing;

  for (std::size_t i = 0; i + 1 < input_path.size(); ++i) {
    const auto& seg_start = input_path[i];
    const auto& seg_end = input_path[i + 1];
    double dx = seg_end.x - seg_start.x;
    double dy = seg_end.y - seg_start.y;
    double seg_len = std::sqrt(dx * dx + dy * dy);
    if (seg_len <= 1e-9) continue;  // 跳过零长度段

    // 在当前段内按固定间距投放采样点
    double along = dist_to_next_sample;
    while (along < seg_len) {
      double t = along / seg_len;
      out.push_back({seg_start.x + t * dx, seg_start.y + t * dy});
      along += spacing;
    }
    // 结转下一段剩余的采样距离
    dist_to_next_sample = along - seg_len;
    if (dist_to_next_sample <= 1e-9) dist_to_next_sample = spacing;
  }

  // 保留精确的终点（避免与最后一个采样点重复）
  if (std::hypot(out.back().x - input_path.back().x,
                 out.back().y - input_path.back().y) > 1e-6) {
    out.push_back(input_path.back());
  }
  return out;
}

// ==================== PathShortcutter ====================

/**
 * @brief 视线快捷化（贪心）
 * 从锚点出发向后寻找"视线可达"的最远点，直接跳转，逐步裁掉 A* 锯齿拐角
 */
PointPath2D PathShortcutter::shortcut(
  const PointPath2D& raw_path,
  const std::function<bool(const PathPoint2D&, const PathPoint2D&)>& has_line_of_sight) const
{
  if (raw_path.size() <= 2) return raw_path;

  PointPath2D out;
  out.reserve(raw_path.size());
  out.push_back(raw_path.front());

  std::size_t anchor = 0;
  while (anchor < raw_path.size() - 1) {
    std::size_t furthest = anchor + 1;
    for (std::size_t candidate = anchor + 1; candidate < raw_path.size(); ++candidate) {
      if (!has_line_of_sight(raw_path[anchor], raw_path[candidate])) break;
      furthest = candidate;
    }
    out.push_back(raw_path[furthest]);
    anchor = furthest;
  }
  return out;
}

// ==================== SplinePathSmoother ====================

/**
 * @brief 弧长参数化：每个路径点的累计弧长 s(0)=0, s(i)=s(i-1)+|Δp|
 */
std::vector<double> SplinePathSmoother::buildArcLengthParameter(const PointPath2D& points) const {
  std::vector<double> s(points.size(), 0.0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    double dx = points[i].x - points[i - 1].x;
    double dy = points[i].y - points[i - 1].y;
    s[i] = s[i - 1] + std::sqrt(dx * dx + dy * dy);
  }
  return s;
}

std::vector<double> SplinePathSmoother::extractXPathValues(const PointPath2D& points) const {
  std::vector<double> v;
  v.reserve(points.size());
  for (const auto& p : points) v.push_back(p.x);
  return v;
}

std::vector<double> SplinePathSmoother::extractYPathValues(const PointPath2D& points) const {
  std::vector<double> v;
  v.reserve(points.size());
  for (const auto& p : points) v.push_back(p.y);
  return v;
}

/**
 * @brief 自然三次样条：解三对角方程组求各节点的二阶导数（自然边界条件 M0=Mn-1=0）
 * 采用追赶法（Thomas 算法），O(n) 复杂度
 */
std::vector<double> SplinePathSmoother::solveNaturalCubicSecondDerivatives(
  const std::vector<double>& s, const std::vector<double>& v) const
{
  const std::size_t n = v.size();
  std::vector<double> m(n, 0.0);
  if (n < 3) return m;

  const std::size_t interior = n - 2;
  std::vector<double> a(interior, 0.0), b(interior, 0.0), c(interior, 0.0), d(interior, 0.0);

  for (std::size_t i = 0; i < interior; ++i) {
    const std::size_t k = i + 1;  // 内节点索引
    double h_prev = s[k] - s[k - 1];
    double h_next = s[k + 1] - s[k];
    if (h_prev <= 1e-9 || h_next <= 1e-9) return m;  // 退化参数，放弃平滑

    a[i] = h_prev;
    b[i] = 2.0 * (h_prev + h_next);
    c[i] = h_next;
    d[i] = 6.0 * ((v[k + 1] - v[k]) / h_next - (v[k] - v[k - 1]) / h_prev);
  }

  // 追：消元
  for (std::size_t i = 1; i < interior; ++i) {
    double factor = a[i] / b[i - 1];
    b[i] -= factor * c[i - 1];
    d[i] -= factor * d[i - 1];
  }

  // 赶：回代
  m[n - 2] = d.back() / b.back();
  for (std::size_t i = interior - 1; i > 0; --i) {
    m[i] = (d[i - 1] - c[i - 1] * m[i + 1]) / b[i - 1];
  }
  return m;
}

/**
 * @brief 三次样条求值：给定弧长 s_query，定位所在分段后按标准样条公式插值
 */
double SplinePathSmoother::evaluateNaturalCubicSpline(
  const std::vector<double>& s, const std::vector<double>& v,
  const std::vector<double>& m, double s_query) const
{
  auto upper = std::upper_bound(s.begin(), s.end(), s_query);
  std::size_t i = static_cast<std::size_t>(
    std::max<std::ptrdiff_t>(0, std::distance(s.begin(), upper) - 1));
  i = std::min(i, s.size() - 2);

  double s0 = s[i], s1 = s[i + 1];
  double h = s1 - s0;
  if (h <= 1e-9) return v[i];

  double A = s1 - s_query;  // 到段右端距离
  double B = s_query - s0;  // 到段左端距离
  return m[i] * A * A * A / (6.0 * h) +
         m[i + 1] * B * B * B / (6.0 * h) +
         (v[i] - m[i] * h * h / 6.0) * (A / h) +
         (v[i + 1] - m[i + 1] * h * h / 6.0) * (B / h);
}

/**
 * @brief 样条平滑主流程：参数化 → 解样条 → 固定间距重采样 → 逐点碰撞检测（失败回退原路径）
 */
SmoothingResult2D SplinePathSmoother::smooth(
  const PointPath2D& base_path,
  double sample_spacing,
  double map_resolution,
  const std::function<bool(const PathPoint2D&)>& is_point_in_free_space) const
{
  SmoothingResult2D result;
  result.path = base_path;

  if (base_path.size() < 3) return result;

  const std::vector<double> s = buildArcLengthParameter(base_path);
  if (s.size() < 3 || s.back() <= 1e-6) return result;

  const std::vector<double> xs = extractXPathValues(base_path);
  const std::vector<double> ys = extractYPathValues(base_path);
  const std::vector<double> mx = solveNaturalCubicSecondDerivatives(s, xs);
  const std::vector<double> my = solveNaturalCubicSecondDerivatives(s, ys);

  const double total_length = s.back();
  // 最小采样间距不低于半格分辨率，避免输出点密度超过地图表达能力
  const double spacing = std::max(sample_spacing, map_resolution * 0.5);

  PointPath2D sampled;
  sampled.reserve(
    std::max<std::size_t>(base_path.size(),
      static_cast<std::size_t>(std::ceil(total_length / spacing)) + 1U));
  sampled.push_back(base_path.front());

  for (double d = spacing; d < total_length; d += spacing) {
    sampled.push_back({
      evaluateNaturalCubicSpline(s, xs, mx, d),
      evaluateNaturalCubicSpline(s, ys, my, d)});
  }
  sampled.push_back(base_path.back());

  // 碰撞检测：样条可能切进障碍物，任一点不自由则回退原路径
  for (const auto& p : sampled) {
    if (!is_point_in_free_space(p)) {
      result.used_collision_fallback = true;
      return result;
    }
  }

  result.path = sampled;
  return result;
}

}  // namespace robot
