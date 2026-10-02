// Derived leaf-size sweep for thesis Table VIII, from ONE saved dense cloud.
// For each leaf L: PCL VoxelGrid(L) on the cloud -> N points; ColorOcTree(L) built exactly like
// PointCloudMapping::InsertIntoOctomap (occupied endpoints, lazy eval, integrateNodeColor), then
// updateInnerOccupancy+prune, write .ot and the occupancy-only .bt. TSV columns:
//   L  points  pcd_bin_xyzrgb  pcd_ascii_est  ot  ot_nodes  bt
// This approximates (it is not) a fresh SLAM run at Dense.resolution=L: per-keyframe filtering and
// merging are not replayed.
// Build: as size_report.cc plus -lpcl_filters.   Usage: leaf_sweep cloud.pcd L1 L2 ...
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <sys/stat.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
static long long fsz(const std::string &p) { struct stat st; return stat(p.c_str(), &st) ? -1 : (long long)st.st_size; }
typedef pcl::PointXYZRGB P;
static long long asciiEst(const pcl::PointCloud<P> &c)
{
    const long long n = (long long)c.size();
    pcl::PointCloud<P> smp;
    const long long step = n > 200000 ? n / 200000 : 1;
    for (long long k = 0; k < n; k += step) smp.push_back(c.points[k]);
    const std::string t = "/tmp/sw_smp.pcd";
    pcl::io::savePCDFileASCII(t, smp);
    FILE *f = fopen(t.c_str(), "rb"); char buf[1024]; size_t r = fread(buf, 1, sizeof buf, f); fclose(f);
    long long hdr = (long long)(std::string(buf, r).find("DATA ascii\n") + 11);
    long long a = fsz(t); std::remove(t.c_str());
    return (long long)(double(a - hdr) / smp.size() * n) + hdr + 10;
}
int main(int argc, char **argv)
{
    pcl::PointCloud<P>::Ptr c(new pcl::PointCloud<P>);
    if (pcl::io::loadPCDFile(argv[1], *c) < 0) return 1;
    for (int i = 2; i < argc; ++i)
    {
        const float L = (float)atof(argv[i]);
        pcl::PointCloud<P>::Ptr d(new pcl::PointCloud<P>);
        pcl::VoxelGrid<P> vg; vg.setLeafSize(L, L, L); vg.setInputCloud(c); vg.filter(*d);
        octomap::ColorOcTree tree(L);
        for (const auto &p : d->points) tree.updateNode(p.x, p.y, p.z, true, true);
        for (const auto &p : d->points) tree.integrateNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);
        tree.updateInnerOccupancy(); tree.prune();
        tree.write("/tmp/sw.ot"); tree.writeBinary("/tmp/sw.bt");
        const long long n = (long long)d->size();
        // exact binary size: save the filtered cloud
        pcl::io::savePCDFileBinary("/tmp/sw.pcd", *d);
        std::printf("%g\t%lld\t%lld\t%lld\t%lld\t%lld\t%lld\n", L, n, fsz("/tmp/sw.pcd"), asciiEst(*d), fsz("/tmp/sw.ot"), (long long)tree.size(), fsz("/tmp/sw.bt"));
        std::fflush(stdout);
        std::remove("/tmp/sw.ot"); std::remove("/tmp/sw.bt"); std::remove("/tmp/sw.pcd");
    }
    return 0;
}
