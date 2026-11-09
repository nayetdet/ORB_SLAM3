// Applies the Dense.voxelSafe global voxel filter (src/PointCloudMapping.cc) to a saved cloud,
// offline, and reports what it did: points in/out, slabs, wall time, memory.
//
//   voxel_safe_cloud <cloud.pcd> <leaf> [--legacy] [--out filtered.raw] [--replay K]
//
// Default: the voxelSafe path (voxelFilterSafe, the code VoxelFilterGlobal runs with
// voxelSafe=1). --legacy: the call VoxelFilterGlobal makes with voxelSafe=0, for the cost of
// the do-nothing path pcl takes on an overflowing grid. --out: writes the result as raw
// float32 x,y,z,rgb-bits (16 bytes per point) for evaluation/tests/voxel_count_check.py.
//
// --replay K: instead of one call, replays a run's global filtering. The cloud is cut into K
// consecutive chunks (K = keyframes / 10, one call per 10 keyframes); for each, the real
// Impl::MergeAppend appends the chunk and the real Impl::VoxelFilterGlobal filters the world
// cloud, as the dense thread does with Dense.voxelSafe = 1 (or 0 with --legacy). The cloud
// being replayed is itself a legacy run's output, so the stream is an approximation of the
// raw keyframe clouds (it already lost whatever the first non-overflowing calls merged).
//
// The input is only read. Memory is the resident set sampled every 2 ms while the filter runs.
// Build/run: run_voxel_safe_cloud.sh.

#include <opencv2/core/core.hpp>
#include <string>
#define private public
#include "PointCloudMapping.h"
#undef private
#include "PointCloudMapping.cc"

#include <atomic>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <thread>
#include <unistd.h>

using namespace ORB_SLAM3;

static double rssGB()
{
    std::ifstream f("/proc/self/statm");
    long total = 0, resident = 0;
    f >> total >> resident;
    return static_cast<double>(resident) * static_cast<double>(sysconf(_SC_PAGESIZE)) / 1e9;
}

static double hwmGB()
{
    std::ifstream f("/proc/self/status");
    std::string line;
    while (std::getline(f, line))
        if (line.compare(0, 6, "VmHWM:") == 0) return std::stod(line.substr(6)) * 1024.0 / 1e9;
    return -1.0;
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::fprintf(stderr, "usage: %s cloud.pcd leaf [--legacy] [--out filtered.raw]\n", argv[0]);
        return 2;
    }
    const std::string pcd = argv[1];
    const float leaf = std::stof(argv[2]);
    bool legacy = false;
    int replay = 0;
    std::string outPath;
    for (int i = 3; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--legacy") legacy = true;
        else if (std::string(argv[i]) == "--out" && i + 1 < argc) outPath = argv[++i];
        else if (std::string(argv[i]) == "--replay" && i + 1 < argc) replay = std::stoi(argv[++i]);
    }
    using clock = std::chrono::steady_clock;
    auto sec = [](clock::time_point a, clock::time_point b) { return std::chrono::duration<double>(b - a).count(); };

    PointCloudT::Ptr cloud(new PointCloudT());
    auto t0 = clock::now();
    if (pcl::io::loadPCDFile<PointT>(pcd, *cloud) < 0) { std::fprintf(stderr, "cannot read %s\n", pcd.c_str()); return 1; }
    std::printf("loaded %s: %zu points (sizeof(PointXYZRGB) = %zu B in memory) in %.1f s, RSS %.2f GB\n", pcd.c_str(),
                cloud->size(), sizeof(PointT), sec(t0, clock::now()), rssGB());

    const VoxelExtent ext = voxelExtent(*cloud, 1.0f / leaf);
    std::printf("leaf %.4g m: pcl's dx,dy,dz = %lld, %lld, %lld -> %.3e cells; single pass %s (limit %.3e)\n", leaf,
                static_cast<long long>(ext.d[0]), static_cast<long long>(ext.d[1]), static_cast<long long>(ext.d[2]),
                double(ext.d[0]) * ext.d[1] * ext.d[2], ext.overflows ? "REFUSED by pcl (overflow)" : "accepted",
                double(std::numeric_limits<std::int32_t>::max()));

    // resident-set sampler
    const double baseline = rssGB();
    std::atomic<bool> stop{false};
    std::atomic<double> peak{baseline};
    std::thread sampler([&] {
        while (!stop)
        {
            const double r = rssGB();
            if (r > peak) peak = r;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });

    if (replay > 0)
    {
        PointCloudMapping::Config cfg;
        cfg.enabled = true; cfg.octomapEnabled = false; cfg.offline = true;   // offline, no spill: no thread
        cfg.resolution = leaf;
        cfg.voxelSafe = !legacy;
        std::ostringstream banner;
        std::streambuf *old = std::cout.rdbuf(banner.rdbuf());
        PointCloudMapping::Impl impl(cfg, System::RGBD);
        std::cout.rdbuf(old);

        const size_t n = cloud->size();
        double total = 0.0, worst = 0.0;
        int worstAt = 0;
        std::printf("replay of %d dense-thread calls (%s), %zu points in %d chunks of ~%zu\n", replay,
                    legacy ? "voxelSafe=0" : "voxelSafe=1", n, replay, n / replay);
        for (int i = 0; i < replay; ++i)
        {
            const size_t a = n * static_cast<size_t>(i) / replay, b = n * static_cast<size_t>(i + 1) / replay;
            PointCloudT::Ptr chunk(new PointCloudT());
            chunk->points.assign(cloud->points.begin() + static_cast<std::ptrdiff_t>(a),
                                 cloud->points.begin() + static_cast<std::ptrdiff_t>(b));
            chunk->width = static_cast<std::uint32_t>(chunk->size());
            chunk->height = 1;
            chunk->is_dense = true;
            impl.MergeAppend(chunk, std::vector<float>(chunk->size(), 1.0f));
            const size_t before = impl.mpGlobalCloud->size();
            const auto t = clock::now();
            impl.VoxelFilterGlobal();
            const double dt = sec(t, clock::now());
            total += dt;
            if (dt > worst) { worst = dt; worstAt = i + 1; }
            if ((i + 1) % std::max(1, replay / 12) == 0 || i + 1 == replay)
                std::printf("  call %3d: %10zu -> %10zu points, %6.2f s   (overflow so far %zu, slabbed %zu)\n", i + 1,
                            before, impl.mpGlobalCloud->size(), dt, size_t(impl.mnVoxelOverflow), size_t(impl.mnVoxelSlabbed));
        }
        stop = true;
        sampler.join();
        std::printf("  Global voxel filter (voxelSafe=%d): %zu calls, %zu would overflow pcl's int32 grid, %zu handled by slabs, "
                    "%zu left unfiltered, %.2f s\n", !legacy, size_t(impl.mnVoxelCalls), size_t(impl.mnVoxelOverflow),
                    size_t(impl.mnVoxelSlabbed), size_t(impl.mnVoxelUnslabbed), impl.mVoxelSeconds.load());
        std::printf("  total filter time %.1f s over %d calls (mean %.2f s, worst %.2f s at call %d); final cloud %zu points\n",
                    total, replay, total / replay, worst, worstAt, impl.mpGlobalCloud->size());
        std::printf("  RSS before the replay %.2f GB, peak %.2f GB (includes the replayed input cloud); VmHWM %.2f GB\n",
                    baseline, peak.load(), hwmGB());
        return 0;
    }

    PointCloudT out;
    size_t nSlabs = 0;
    std::string path;
    t0 = clock::now();
    if (legacy)
    {
        pcl::VoxelGrid<PointT> voxel;
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.setInputCloud(cloud);
        voxel.filter(out);
        path = voxel.getNrDivisions()[0] == 0 ? "legacy single pass: pcl refused, cloud returned unfiltered"
                                              : "legacy single pass: pcl filtered";
    }
    else
    {
        const VoxelPath p = voxelFilterSafe(cloud, leaf, kVoxelSlabCells, out, &nSlabs);
        path = p == VoxelPath::SinglePass ? "voxelSafe: single pass (grid fits)"
             : p == VoxelPath::Slabs      ? "voxelSafe: slabs"
                                          : "voxelSafe: NOT slabbable, left unfiltered";
    }
    const double wall = sec(t0, clock::now());
    stop = true;
    sampler.join();

    std::printf("%s\n", path.c_str());
    std::printf("  slabs used            : %zu (budget %lld cells per slab)\n", nSlabs, static_cast<long long>(kVoxelSlabCells));
    std::printf("  points in -> out      : %zu -> %zu  (out/in = %.4f)\n", cloud->size(), out.size(),
                double(out.size()) / double(cloud->size()));
    std::printf("  wall time             : %.2f s\n", wall);
    std::printf("  RSS before the filter : %.2f GB; peak during it: %.2f GB (+%.2f GB); process VmHWM %.2f GB\n",
                baseline, peak.load(), peak.load() - baseline, hwmGB());

    // One point per voxel? Re-bin the output (cells from its own bounding box, linear int64 keys).
    {
        const float inv = 1.0f / leaf;
        std::int64_t lo[3] = {INT64_MAX, INT64_MAX, INT64_MAX}, hi[3] = {INT64_MIN, INT64_MIN, INT64_MIN};
        for (const auto &p : out.points)
            for (int a = 0; a < 3; ++a)
            {
                const std::int64_t v = static_cast<std::int64_t>(std::floor(p.data[a] * inv));
                lo[a] = std::min(lo[a], v);
                hi[a] = std::max(hi[a], v);
            }
        const std::int64_t d1 = hi[1] - lo[1] + 1, d2 = hi[2] - lo[2] + 1;
        std::vector<std::int64_t> keys;
        keys.reserve(out.size());
        for (const auto &p : out.points)
        {
            const std::int64_t i = static_cast<std::int64_t>(std::floor(p.x * inv)) - lo[0];
            const std::int64_t j = static_cast<std::int64_t>(std::floor(p.y * inv)) - lo[1];
            const std::int64_t k = static_cast<std::int64_t>(std::floor(p.z * inv)) - lo[2];
            keys.push_back((i * d1 + j) * d2 + k);
        }
        std::sort(keys.begin(), keys.end());
        const size_t distinct = static_cast<size_t>(std::unique(keys.begin(), keys.end()) - keys.begin());
        std::printf("  output re-binned      : %zu points in %zu distinct voxels -> %.4f output points per occupied voxel\n",
                    out.size(), distinct, double(out.size()) / double(distinct));
    }

    if (!outPath.empty())
    {
        std::ofstream f(outPath, std::ios::binary);
        for (const auto &p : out.points)
        {
            float rec[4];
            std::memcpy(rec, &p.x, 12);
            std::memcpy(&rec[3], &p.rgba, 4);
            f.write(reinterpret_cast<const char *>(rec), 16);
        }
        std::printf("  wrote %zu points to %s\n", out.size(), outPath.c_str());
    }
    return 0;
}
