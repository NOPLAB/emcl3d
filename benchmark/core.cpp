// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#include "emcl3d/particle_filter.hpp"
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <omp.h>

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  omp_set_dynamic(0);
  auto map = std::make_shared<emcl3d::Map>();
  map->load(argv[1], 0.2);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(argv[1], cloud) < 0) return 2;
  std::vector<Eigen::Vector3d> source;
  for (const auto &p : cloud) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
      source.push_back(p.getVector3fMap().cast<double>());
  }
  if (source.empty()) return 2;
  const Eigen::Vector3d origin = source.front();
  std::vector<Eigen::Vector3d> local;
  for (const auto &p : source)
    if ((p - origin).norm() < 70) local.push_back(p - origin);
  std::cout << std::setprecision(12);
  for (size_t count : {size_t(50), size_t(2000)}) {
    if (local.size() < count) return 2;
    std::vector<Eigen::Vector3d> points;
    for (size_t i = 0; i < count; ++i)
      points.push_back(local[i * local.size() / count]);
    for (bool distances : {false, true}) {
      double checksum = 0;
      const auto start = std::chrono::steady_clock::now();
      #pragma omp parallel for num_threads(2) reduction(+:checksum)
      for (int particle = 0; particle < 50; ++particle) {
        const Eigen::Vector3d offset(0.001 * particle, 0.002 * particle, 0.003 * particle);
        for (const auto &point : points) {
          const Eigen::Vector3d query = point + origin + offset;
          checksum += distances ? map->distance(query) : map->contains(query, 0.5);
        }
      }
      std::cout << "phase=" << (distances ? "distance" : "contains")
                << " points=" << count << " ms="
                << std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - start).count()
                << " checksum=" << checksum << std::endl;
    }
    for (int round = 0; round < 5; ++round) {
      emcl3d::Config config;
      config.num_particles = 50;
      config.num_threads = 2;
      // A fixed seed and identical observations exercise the same trajectory.
      emcl3d::ParticleFilter filter(config, map);
      auto pose = emcl3d::Pose::Identity();
      pose.translation() = origin;
      filter.initialize(pose, emcl3d::Matrix6::Identity() * 0.01);
      for (int i = 0; i < 5; ++i)
        filter.update(emcl3d::Pose::Identity(), points);
      std::vector<double> times;
      double checksum = 0;
      int resets = 0;
      const auto cpu_start = std::clock();
      for (int i = 0; i < 30; ++i) {
        const auto start = std::chrono::steady_clock::now();
        const auto estimate = filter.update(emcl3d::Pose::Identity(), points);
        times.push_back(std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count());
        checksum += estimate.pose.matrix().sum() + estimate.covariance.sum() +
                    estimate.alpha + estimate.ess;
        resets += estimate.reset;
      }
      const double cpu_ms = 1000.0 * (std::clock() - cpu_start) / CLOCKS_PER_SEC;
      std::sort(times.begin(), times.end());
      std::cout << "points=" << count << " round=" << round
                << " median_ms=" << times[15] << " p95_ms=" << times[28]
                << " max_ms=" << times.back()
                << " cpu_ms_per_update=" << cpu_ms / 30
                << " checksum=" << checksum << " resets=" << resets << std::endl;
    }
  }
}
