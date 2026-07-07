#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace global_reloc {

// We use PCL's built-in PointNormal (xyz + normal + curvature). It is fully
// registered with every accessor PCL's algorithms expect
// (getNormalVector4fMap, etc.) and is precompiled in the PCL libraries, so we
// avoid the custom-point-type template instantiation / registration headaches.
// Kept as a typedef for source readability.
using PointXYZRN = pcl::PointNormal;

}  // namespace global_reloc
