// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#pragma once

namespace emcl3d::detail {
inline float MapIndex::lowerBound(const Node &node, const Eigen::Array4f &query) {
  const Eigen::Array4f gap = (node.low - query).max(0) + (query - node.high).max(0);
  return gap.square().sum();
}
inline void MapIndex::nearest(size_t id, const Eigen::Array4f &query, float &best) const {
  const auto &node = nodes[id];
  if (node.count) {
    for (size_t i = node.first; i < node.first + node.count; ++i) {
      const auto &b = blocks[i];
      const Eigen::Array<float, 1, 4> squared =
          (b.row(0) - query[0]).square() + (b.row(1) - query[1]).square() +
          (b.row(2) - query[2]).square();
      best = std::min(best, squared.minCoeff());
    }
    return;
  }
  size_t first = node.left, second = node.right;
  float a = lowerBound(nodes[first], query), b = lowerBound(nodes[second], query);
  if (b < a) { std::swap(a, b); std::swap(first, second); }
  // Conservative slack prevents float box roundoff from pruning a tied neighbor.
  constexpr float slack = 1 + 16 * std::numeric_limits<float>::epsilon();
  if (!std::isfinite(a) || a <= best * slack) nearest(first, query, best);
  if (!std::isfinite(b) || b <= best * slack) nearest(second, query, best);
}
inline float MapIndex::squaredDistance(const Eigen::Array4f &query, float limit) const {
  float best = limit;
  nearest(0, query, best);
  return best < limit ? best : std::numeric_limits<float>::infinity();
}
}  // namespace emcl3d::detail
