// Included in the pinned grid_map.cpp by patch_ego.py. Read-only, on-demand.
// The pinned executable uses ros::spin(): this callback cannot interleave map updates.
bool GridMap::v22Query(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
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
  if (size.cast<double>().prod() > 4000000)
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
  out << std::setprecision(12)
      << "{\"resolution_m\":" << mp_.resolution_
      << ",\"stamp_s\":" << v22_cloud_stamp_.toSec()
      << ",\"version\":" << v22_map_version_
      << ",\"lower\":[" << lo.x() << ',' << lo.y() << ',' << lo.z() << ']'
      << ",\"upper\":[" << hi.x() << ',' << hi.y() << ',' << hi.z() << ']'
      << ",\"ground_m\":" << (mp_.enable_virtual_walll_ ? mp_.virtual_ground_ + mp_.obstacles_inflation_ : lo.z()*mp_.resolution_)
      << ",\"ceiling_m\":" << (mp_.enable_virtual_walll_ ? mp_.virtual_ceil_ - mp_.obstacles_inflation_ : (hi.z()+1)*mp_.resolution_)
      << ",\"occupied\":[" << raw.str() << "],\"inflated\":[" << inflated.str() << "]}";
  res.success = true;
  res.message = out.str();
  return true;
}
