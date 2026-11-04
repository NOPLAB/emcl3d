// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#pragma once
#include <Eigen/Core>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace emcl3d::test {
class IrregularMapFile {
  std::unique_ptr<FILE, decltype(&std::fclose)> file_{std::tmpfile(), &std::fclose};
public:
  std::vector<Eigen::Vector3f> points;
  IrregularMapFile() {
    if (!file_) throw std::runtime_error("Cannot create temporary PCD");
    for (int x = 0; x < 8; ++x)
      for (int y = 0; y < 8; ++y)
        for (int z = 0; z < 8; ++z) {
          if (x == 7 && y == 7 && z == 7) continue;
          points.emplace_back(x + 0.17f * ((y * 13 + z * 7) % 5),
              y + 0.13f * ((x * 11 + z * 3) % 7), z + 0.19f * ((x * 5 + y * 3) % 3));
        }
    std::fprintf(file_.get(),
        "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH %zu\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %zu\nDATA ascii\n",
        points.size(), points.size());
    for (const auto &p : points)
      std::fprintf(file_.get(), "%.9g %.9g %.9g\n", p.x(), p.y(), p.z());
    if (std::fflush(file_.get()) != 0) throw std::runtime_error("Cannot flush temporary PCD");
  }
  std::string path() const { return "/proc/self/fd/" + std::to_string(::fileno(file_.get())); }
};
}  // namespace emcl3d::test
