// Offline file-size report for thesis Tables VIII/IX (D2 in melhorias.md).
// For each <cloud.pcd> <octomap.ot> pair prints one TSV row:
//   label  points  pcd_bin_xyzrgb  pcd_bin_xyz  pcd_ascii_est  ot  ot_nodes  bt
// pcd_ascii_est is extrapolated from the exact ASCII size of an evenly spaced
// sample of <= 200k points written by PCL itself (header added exactly).
// .bt is the occupancy-only octomap::OcTree::writeBinary of the loaded .ot.
// Build: g++ -O2 -std=c++17 size_report.cc -I/usr/include/pcl-1.15 -I/usr/include/eigen3 \
//        -lpcl_io -lpcl_common -loctomap -loctomath -o size_report
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <sys/stat.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

static long long fsz(const std::string &p) { struct stat st; return stat(p.c_str(), &st) ? -1 : (long long)st.st_size; }

int main(int argc, char **argv)
{
    // args: label pcd ot [label pcd ot ...]
    for (int i = 1; i + 2 < argc + 0; i += 3)
    {
        std::string label = argv[i], pcd = argv[i + 1], ot = argv[i + 2];
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr c(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPCDFile(pcd, *c) < 0) { std::cerr << "cannot load " << pcd << "\n"; continue; }
        const long long n = (long long)c->size();
        const long long binXyzrgb = fsz(pcd);

        pcl::PointCloud<pcl::PointXYZ> xyz;
        pcl::PointCloud<pcl::PointXYZRGB> smp;
        const long long step = n > 200000 ? n / 200000 : 1;
        for (long long k = 0; k < n; k += step) smp.push_back(c->points[k]);
        const std::string t1 = "/tmp/sr_smp.pcd", t2 = "/tmp/sr_xyz.pcd";
        pcl::io::savePCDFileASCII(t1, smp);
        const long long a = fsz(t1);
        // header of the sample: find "DATA ascii\n"
        long long hdr = 0; { FILE *f = fopen(t1.c_str(), "rb"); char buf[1024]; size_t r = fread(buf, 1, sizeof buf, f); fclose(f);
          std::string s(buf, r); size_t p = s.find("DATA ascii\n"); hdr = (long long)(p + 11); }
        const double perPt = double(a - hdr) / double(smp.size());
        const long long asciiEst = (long long)(perPt * n) + hdr + 10;  // header differs by a few digits only
        std::remove(t1.c_str());
        // binary XYZ-only size: header (~ 150 B) + 12 B/point; measured on a 1-point file for the header
        xyz.push_back(pcl::PointXYZ(0, 0, 0));
        pcl::io::savePCDFileBinary(t2, xyz);
        const long long hdrXyz = fsz(t2) - 12;
        std::remove(t2.c_str());
        const long long binXyz = hdrXyz + 12LL * n + 8;

        octomap::AbstractOcTree *t = octomap::AbstractOcTree::read(ot);
        long long nodes = -1, bt = -1;
        if (t)
        {
            nodes = (long long)t->size();
            octomap::ColorOcTree *ct = dynamic_cast<octomap::ColorOcTree *>(t);
            if (ct)
            {
                // writeBinary on the loaded tree; ColorOcTree -> occupancy only
                const std::string b = "/tmp/sr_tree.bt";
                if (ct->writeBinary(b)) bt = fsz(b);
                std::remove(b.c_str());
            }
            delete t;
        }
        std::printf("%s\t%lld\t%lld\t%lld\t%lld\t%lld\t%lld\t%lld\n", label.c_str(), n, binXyzrgb, binXyz, asciiEst, fsz(ot), nodes, bt);
        std::fflush(stdout);
    }
    return 0;
}
