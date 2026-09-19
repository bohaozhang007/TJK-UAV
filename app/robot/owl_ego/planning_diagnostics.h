#pragma once
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>

inline std::string v22Vector(const Eigen::Vector3d &point)
{
  std::ostringstream out;
  out << std::setprecision(12) << '[';
  for (int i = 0; i < 3; ++i)
  {
    if (i) out << ',';
    if (std::isfinite(point[i])) out << point[i]; else out << "null";
  }
  out << ']';
  return out.str();
}

struct V22PlanningDiagnostics
{
  bool active = false;
  size_t omitted = 0;
  std::vector<std::string> events;

  void clear() { events.clear(); omitted = 0; }

  void add(const std::string &stage, const std::string &reason, double elapsed = 0,
           const std::string &detail = "")
  {
    if (!active) return;
    if (events.size() >= 128) { ++omitted; return; }
    std::ostringstream out;
    out << std::setprecision(12) << "{\"stage\":\"" << stage << "\",\"reason\":\"" << reason
        << "\",\"elapsed_s\":" << elapsed << detail << '}';
    events.push_back(out.str());
  }

  std::string json() const
  {
    std::ostringstream out;
    out << '[';
    for (size_t i = 0; i < events.size(); ++i) { if (i) out << ','; out << events[i]; }
    out << ']';
    return out.str();
  }
};
