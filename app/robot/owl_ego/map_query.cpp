// Included in the pinned grid_map.cpp by patch_ego.py. Read-only, on-demand.
// The pinned executable uses ros::spin(): this callback cannot interleave map updates.
#include <chrono>
bool GridMap::v22Query(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
  const auto started = std::chrono::steady_clock::now();
  const double age = (ros::Time::now() - v22_cloud_stamp_).toSec();
  if (!mp_.have_initialized_ || !md_.has_odom_ || v22_cloud_stamp_.isZero() || age < 0 || age > 0.5)
  {
    res.success = false;
    res.message = "map has no fresh integrated cloud";
    return true;
  }
  // Exclude the upper alias: this ring has 2*range cells, not 2*range+1.
  const Eigen::Vector3i lo = md_.ringbuffer_lowbound3i_;
  const Eigen::Vector3i hi = lo + md_.ringbuffer_size3i_ - Eigen::Vector3i::Ones();
  const Eigen::Vector3i size = hi - lo + Eigen::Vector3i::Ones();
  if (size.cast<double>().prod() > 8000000)
  {
    res.success = false;
    res.message = "map query exceeds voxel budget";
    return true;
  }
  std::ostringstream raw, inflated, out;
  size_t raw_count = 0, inflated_count = 0;
  for (int x = lo.x(); x <= hi.x(); ++x)
    for (int y = lo.y(); y <= hi.y(); ++y)
      for (int z = lo.z(); z <= hi.z(); ++z)
      {
        const Eigen::Vector3i id(x, y, z);
        if (md_.occupancy_buffer_[globalIdx2BufIdx(id)] >= mp_.min_occupancy_log_)
        {
          if (raw_count++) raw << ',';
          raw << '[' << x << ',' << y << ',' << z << ']';
        }
        if (md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(id)])
        {
          if (inflated_count++) inflated << ',';
          inflated << '[' << x << ',' << y << ',' << z << ']';
        }
        if (raw_count + inflated_count > 300000)
        {
          res.success = false;
          res.message = "map query exceeds occupied-cell budget";
          return true;
        }
      }
  const double scan_encode_s = std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
  out << std::setprecision(12)
      << "{\"resolution_m\":" << mp_.resolution_
      << ",\"stamp_s\":" << v22_cloud_stamp_.toSec()
      << ",\"version\":" << v22_map_version_
      << ",\"lower\":[" << lo.x() << ',' << lo.y() << ',' << lo.z() << ']'
      << ",\"upper\":[" << hi.x() << ',' << hi.y() << ',' << hi.z() << ']'
      << ",\"ground_m\":" << (mp_.enable_virtual_walll_ ? mp_.virtual_ground_ + mp_.obstacles_inflation_ : lo.z()*mp_.resolution_)
      << ",\"ceiling_m\":" << (mp_.enable_virtual_walll_ ? mp_.virtual_ceil_ - mp_.obstacles_inflation_ : (hi.z()+1)*mp_.resolution_)
      << ",\"query_metrics\":{\"voxel_count\":" << size.cast<double>().prod()
      << ",\"voxel_budget\":8000000,\"occupied_count\":" << raw_count
      << ",\"inflated_count\":" << inflated_count
      << ",\"scan_encode_s\":" << scan_encode_s << "}"
      << ",\"occupied\":[" << raw.str() << "],\"inflated\":[" << inflated.str() << "]}";
  res.success = true;
  res.message = out.str();
  return true;
}

std::string GridMap::v22PointState(const Eigen::Vector3d &point)
{
  if (!mp_.have_initialized_ || !md_.has_odom_ || v22_cloud_stamp_.isZero())
    return "map_not_ready";
  if (!point.allFinite()) return "nonfinite_point";
  const Eigen::Vector3i id = pos2GlobalIdx(point);
  const Eigen::Vector3i end = md_.ringbuffer_lowbound3i_ + md_.ringbuffer_size3i_;
  if ((id.array() < md_.ringbuffer_lowbound3i_.array()).any() || (id.array() >= end.array()).any())
    return "outside_map";
  if (mp_.enable_virtual_walll_ && (point.z() <= mp_.virtual_ground_ || point.z() >= mp_.virtual_ceil_))
    return "outside_height";
  if (md_.occupancy_buffer_[globalIdx2BufIdx(id)] >= mp_.min_occupancy_log_)
    return "raw_occupied";
  if (md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(id)])
    return "inflated_occupied";
  return "free";
}

std::string GridMap::v22PointInfo(const Eigen::Vector3d &point)
{
  return "{\"position_world_m\":" + v22Vector(point) + ",\"state\":\"" + v22PointState(point) + "\"}";
}

std::string GridMap::v22MapInfo()
{
  if (!mp_.have_initialized_ || !md_.has_odom_ || v22_cloud_stamp_.isZero())
    return "{\"ready\":false}";
  std::ostringstream out;
  out << std::setprecision(12) << "{\"version\":" << v22_map_version_
      << ",\"stamp_s\":" << v22_cloud_stamp_.toSec()
      << ",\"age_s\":" << (ros::Time::now()-v22_cloud_stamp_).toSec()
      << ",\"resolution_m\":" << mp_.resolution_
      << ",\"lower\":" << v22Vector(md_.ringbuffer_lowbound3i_.cast<double>())
      << ",\"upper\":" << v22Vector((md_.ringbuffer_lowbound3i_+md_.ringbuffer_size3i_-Eigen::Vector3i::Ones()).cast<double>())
      << '}';
  return out.str();
}
