// Tests for Dense.voxelSafe (src/PointCloudMapping.cc) and the diagnostics added with it.
//
// The test #includes the .cc so it can reach the file-local helpers (voxelFilterSlabs,
// voxelExtent, PrioBoost) and PointCloudMapping::Impl; the library is linked only for
// the KeyFrame/System symbols the .cc refers to. Build and run: run_voxel_safe_tests.sh.
//
//  (a) slab filter vs plain single-pass pcl::VoxelGrid, on clouds where a single pass is
//      valid: random, negative-only, duplicates, points exactly on (and one ulp either
//      side of) multiples of the leaf, NaN/inf, degenerate shapes. Both are also checked
//      against an independent oracle that bins the points itself.
//  (b) a cloud whose box overflows pcl's grid (650 x 45 x 580 m at 0.1 m): the legacy
//      path warns and returns the input, the safe path returns one point per voxel. The
//      cloud is also written to a raw file so numpy can count its voxels independently.
//  (c) boundary of pcl's refusal test, the unslabbable fallback, counters, the summary
//      line, std::cout formatting and the PrioBoost warning.

#include <opencv2/core/core.hpp>
#include <string>
// Impl and mpImpl are private; only this header is opened up.
#define private public
#include "PointCloudMapping.h"
#undef private
#include "PointCloudMapping.cc"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <fstream>
#include <functional>
#include <random>
#include <sstream>
#include <unordered_map>

using namespace ORB_SLAM3;

static int gChecks = 0, gFailed = 0;

#define CHECK(cond, what)                                                                  \
    do {                                                                                   \
        ++gChecks;                                                                         \
        if (!(cond)) { ++gFailed; std::printf("  FAIL  %s  [%s:%d]\n", what, __FILE__, __LINE__); } \
    } while (0)

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

/** Redirects fd 2 to a temp file for the lifetime of the object. */
struct CaptureStderr
{
    int saved, tmp;
    char path[64];
    CaptureStderr()
    {
        std::snprintf(path, sizeof(path), "/tmp/vs_stderr_XXXXXX");
        tmp = mkstemp(path);
        std::fflush(stderr);
        saved = dup(2);
        dup2(tmp, 2);
    }
    std::string finish()
    {
        std::fflush(stderr);
        dup2(saved, 2);
        close(saved);
        close(tmp);
        std::ifstream f(path);
        std::stringstream ss;
        ss << f.rdbuf();
        std::remove(path);
        return ss.str();
    }
};

static std::string captureCout(const std::function<void()> &fn)
{
    std::ostringstream os;
    std::streambuf *old = std::cout.rdbuf(os.rdbuf());
    fn();
    std::cout.rdbuf(old);
    return os.str();
}

static size_t countOf(const std::string &hay, const std::string &needle)
{
    size_t n = 0, pos = 0;
    while ((pos = hay.find(needle, pos)) != std::string::npos) { ++n; pos += needle.size(); }
    return n;
}

static PointT makePoint(float x, float y, float z, std::mt19937 &rng)
{
    PointT p;
    p.x = x; p.y = y; p.z = z;
    p.r = static_cast<std::uint8_t>(rng() & 255);
    p.g = static_cast<std::uint8_t>(rng() & 255);
    p.b = static_cast<std::uint8_t>(rng() & 255);
    return p;
}

static PointCloudT::Ptr makeCloud(const std::vector<PointT> &pts, bool isDense)
{
    PointCloudT::Ptr c(new PointCloudT());
    c->points.assign(pts.begin(), pts.end());
    c->width = static_cast<std::uint32_t>(pts.size());
    c->height = 1;
    c->is_dense = isDense;
    return c;
}

static std::int64_t packKey(std::int64_t i, std::int64_t j, std::int64_t k)
{
    const std::int64_t off = std::int64_t(1) << 20, mask = (std::int64_t(1) << 21) - 1;
    return (((i + off) & mask) << 42) | (((j + off) & mask) << 21) | ((k + off) & mask);
}

static std::array<std::int64_t, 3> cellOf(const PointT &p, float inv)
{
    return {static_cast<std::int64_t>(std::floor(p.x * inv)),
            static_cast<std::int64_t>(std::floor(p.y * inv)),
            static_cast<std::int64_t>(std::floor(p.z * inv))};
}

/** Independent reference: bins the finite points itself (double precision sums). */
static PointCloudT::Ptr oracle(const PointCloudT &cloud, float leaf)
{
    struct Acc { std::int64_t n = 0; double s[3] = {0, 0, 0}; std::uint64_t c[4] = {0, 0, 0, 0}; };
    const float inv = 1.0f / leaf;
    std::unordered_map<std::int64_t, Acc> cells;
    for (const auto &p : cloud.points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const auto c = cellOf(p, inv);
        Acc &a = cells[packKey(c[0], c[1], c[2])];
        ++a.n;
        a.s[0] += p.x; a.s[1] += p.y; a.s[2] += p.z;
        a.c[0] += p.r; a.c[1] += p.g; a.c[2] += p.b; a.c[3] += p.a;
    }
    PointCloudT::Ptr out(new PointCloudT());
    for (const auto &kv : cells)
    {
        const Acc &a = kv.second;
        PointT p;
        p.x = static_cast<float>(a.s[0] / a.n);
        p.y = static_cast<float>(a.s[1] / a.n);
        p.z = static_cast<float>(a.s[2] / a.n);
        p.r = static_cast<std::uint8_t>(a.c[0] / a.n);   // pcl truncates the mean
        p.g = static_cast<std::uint8_t>(a.c[1] / a.n);
        p.b = static_cast<std::uint8_t>(a.c[2] / a.n);
        p.a = static_cast<std::uint8_t>(a.c[3] / a.n);
        out->push_back(p);
    }
    return out;
}

struct MatchResult
{
    size_t unmatched = 0;      // points of `test` with no partner in `ref`
    double maxDev = 0.0;       // largest centroid deviation among the matched pairs
    bool sameSize = false;
    bool ok() const { return sameSize && unmatched == 0; }
};

/** Pairs every point of `test` with a distinct point of `ref` in the same or a
 *  neighbouring cell, within `tol` per coordinate and with identical colour. */
static MatchResult matchSets(const PointCloudT &ref, const PointCloudT &test, float leaf, double tol)
{
    MatchResult r;
    r.sameSize = (ref.size() == test.size());
    const float inv = 1.0f / leaf;
    std::unordered_multimap<std::int64_t, size_t> grid;
    for (size_t i = 0; i < ref.size(); ++i)
    {
        const auto c = cellOf(ref.points[i], inv);
        grid.emplace(packKey(c[0], c[1], c[2]), i);
    }
    std::vector<char> used(ref.size(), 0);
    for (const auto &p : test.points)
    {
        const auto c = cellOf(p, inv);
        bool found = false;
        for (int di = -1; di <= 1 && !found; ++di)
          for (int dj = -1; dj <= 1 && !found; ++dj)
            for (int dk = -1; dk <= 1 && !found; ++dk)
            {
                auto range = grid.equal_range(packKey(c[0] + di, c[1] + dj, c[2] + dk));
                for (auto it = range.first; it != range.second; ++it)
                {
                    const PointT &q = ref.points[it->second];
                    if (used[it->second]) continue;
                    const double dev = std::max({std::fabs(double(p.x) - q.x), std::fabs(double(p.y) - q.y),
                                                 std::fabs(double(p.z) - q.z)});
                    if (dev > tol || p.r != q.r || p.g != q.g || p.b != q.b || p.a != q.a) continue;
                    used[it->second] = 1;
                    r.maxDev = std::max(r.maxDev, dev);
                    found = true;
                    break;
                }
            }
        if (!found) ++r.unmatched;
    }
    return r;
}

static float ulpAt(float v) { return std::nextafter(std::fabs(v), INFINITY) - std::fabs(v); }

static double toleranceFor(const PointCloudT &cloud, float leaf)
{
    // 1e-4 of the leaf, plus the noise of pcl's own float32 sums at that magnitude (a cell
    // of n points near 50 m carries about sqrt(n) ulps; 650 m has 6e-5 m ulps, so no filter
    // working on float32 coordinates can honour 1e-4 of a 0.1 m leaf there).
    float m = 0.0f;
    for (const auto &p : cloud.points)
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
            m = std::max({m, std::fabs(p.x), std::fabs(p.y), std::fabs(p.z)});
    return 1e-4 * leaf + 8.0 * ulpAt(m);
}

static PointCloudT singlePass(const PointCloudT::Ptr &in, float leaf, bool *accepted)
{
    PointCloudT out;
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(in);
    voxel.filter(out);
    *accepted = (voxel.getNrDivisions()[0] != 0);
    return out;
}

// ---------------------------------------------------------------------------
// (a) slab filter vs single pass
// ---------------------------------------------------------------------------

static void testEquivalence(const char *name, const PointCloudT::Ptr &cloud, float leaf)
{
    const double tol = toleranceFor(*cloud, leaf);
    const VoxelExtent ext = voxelExtent(*cloud, 1.0f / leaf);
    if (ext.nFinite == 0)
    {   // pcl itself is undefined on this input; the slab filter must return an empty cloud
        PointCloudT out;
        size_t n = 99;
        CHECK(voxelFilterSlabs(cloud, leaf, ext, kVoxelSlabCells, out, &n) && out.empty() && n == 0,
              "all-non-finite cloud gives an empty result");
        PointCloudT viaSafe;
        CHECK(voxelFilterSafe(cloud, leaf, kVoxelSlabCells, viaSafe) == VoxelPath::SinglePass && viaSafe.empty(),
              "voxelFilterSafe drops an all-non-finite cloud without calling pcl");
        std::printf("  %-26s leaf %.3f  %7zu pts -> 0 cells | all points non-finite\n", name, leaf, cloud->size());
        return;
    }
    const PointCloudT::Ptr orc = oracle(*cloud, leaf);

    bool accepted = false;
    const PointCloudT ref = singlePass(cloud, leaf, &accepted);
    CHECK(accepted, "single pass is valid for this cloud");

    const MatchResult vsOracle = matchSets(*orc, ref, leaf, tol);
    CHECK(vsOracle.ok(), "oracle agrees with plain pcl::VoxelGrid");
    CHECK(!ext.overflows, "estimate says the grid fits");

    // Noise floor: plain pcl on the same points in another order. Its float sums differ from
    // the original order's by rounding alone, which is all a slab split may change too.
    double devShuffle = 0.0;
    {
        std::vector<PointT> pts(cloud->points.begin(), cloud->points.end());
        std::mt19937 rs(99);
        std::shuffle(pts.begin(), pts.end(), rs);
        bool acc2 = false;
        const PointCloudT shuffled = singlePass(makeCloud(pts, cloud->is_dense), leaf, &acc2);
        const MatchResult ms = matchSets(ref, shuffled, leaf, tol);
        CHECK(acc2 && ms.ok(), "plain pcl on a shuffled copy gives the same cells (noise-floor reference)");
        devShuffle = ms.maxDev;
    }

    // Cut thickness in voxel layers for each trial; maxCells chosen to give it.
    std::int64_t cells[3];
    for (int a = 0; a < 3; ++a)
        cells[a] = static_cast<std::int64_t>(std::floor(ext.hi[a] * (1.0f / leaf))) -
                   static_cast<std::int64_t>(std::floor(ext.lo[a] * (1.0f / leaf))) + 1;
    const int ax = (cells[0] >= cells[1] && cells[0] >= cells[2]) ? 0 : (cells[1] >= cells[2] ? 1 : 2);
    const std::int64_t cross = (cells[(ax + 1) % 3] + 1) * (cells[(ax + 2) % 3] + 1);

    size_t maxSlabs = 0;
    double devOracle = vsOracle.maxDev, devSlab = 0.0;   // largest centroid deviations, metres
    bool exact = true;                                   // slab output == single pass bit for bit
    for (std::int64_t thick : {1, 2, 3, 5, 11, 1000000})
    {
        PointCloudT out;
        size_t nSlabs = 0;
        const bool ok = voxelFilterSlabs(cloud, leaf, ext, (thick + 1) * cross, out, &nSlabs);
        CHECK(ok, "slab filter accepts the cloud");
        if (!ok) continue;
        const MatchResult m1 = matchSets(*orc, out, leaf, tol);
        const MatchResult m2 = matchSets(ref, out, leaf, tol);
        const MatchResult mx = matchSets(ref, out, leaf, 0.0);
        CHECK(m1.ok(), "slab output matches the oracle (centroids, colours, one point per cell)");
        CHECK(m2.ok(), "slab output matches plain pcl::VoxelGrid (centroids, colours)");
        CHECK(out.size() == ref.size(), "same number of voxels as the single pass");
        CHECK(out.height == 1 && out.width == out.size() && out.is_dense, "cloud header is consistent");
        maxSlabs = std::max(maxSlabs, nSlabs);
        devOracle = std::max(devOracle, m1.maxDev);
        devSlab = std::max(devSlab, m2.maxDev);
        exact = exact && mx.ok();
        if (m1.unmatched || m2.unmatched)
            std::printf("  thick=%lld unmatched oracle=%zu single=%zu\n", static_cast<long long>(thick),
                        m1.unmatched, m2.unmatched);
    }
    CHECK(devSlab <= std::max(2.0 * devShuffle, 1e-4 * leaf),
          "slab-vs-single deviation stays within the order-of-summation noise floor (or 1e-4 of the leaf)");
    std::printf("  %-26s leaf %.3f  %7zu pts -> %7zu cells | <= %3zu slabs | centroid dev / leaf: "
                "slab-vs-single %.1e%s, shuffled-pcl-vs-single %.1e, vs oracle %.1e\n",
                name, leaf, cloud->size(), ref.size(), maxSlabs, devSlab / leaf,
                exact ? " (bit-identical)" : "", devShuffle / leaf, devOracle / leaf);
}

static void testSlabEquivalence()
{
    std::printf("(a) slab filter vs plain single-pass pcl::VoxelGrid (and an independent oracle)\n");
    std::mt19937 rng(12345);
    auto uni = [&](float a, float b) { return std::uniform_real_distribution<float>(a, b)(rng); };

    {   // random, mixed signs
        std::vector<PointT> v;
        for (int i = 0; i < 400000; ++i) v.push_back(makePoint(uni(-5, 5), uni(-3, 3), uni(-5, 5), rng));
        testEquivalence("random mixed-sign", makeCloud(v, true), 0.1f);
        testEquivalence("random mixed-sign", makeCloud(v, true), 0.25f);
        testEquivalence("random mixed-sign (0.07)", makeCloud(v, true), 0.07f);
    }
    {   // negative coordinates only, far from the origin
        std::vector<PointT> v;
        for (int i = 0; i < 300000; ++i) v.push_back(makePoint(uni(-48, -40), uni(-30, -27), uni(-9, -3), rng));
        testEquivalence("negative only", makeCloud(v, true), 0.1f);
    }
    {   // exact duplicates, plus pairs a hair apart
        std::vector<PointT> v;
        for (int i = 0; i < 60000; ++i)
        {
            const PointT p = makePoint(uni(-4, 4), uni(-4, 4), uni(-4, 4), rng);
            const int k = 1 + static_cast<int>(rng() % 5);
            for (int j = 0; j < k; ++j) v.push_back(p);
        }
        testEquivalence("duplicates", makeCloud(v, true), 0.1f);
    }
    {   // exactly on multiples of the leaf, and one ulp either side of them
        std::vector<PointT> v;
        for (float leaf : {0.1f, 0.3f})
            for (int i = 0; i < 40000; ++i)
            {
                const float f[3] = {leaf * static_cast<float>(static_cast<int>(rng() % 81) - 40),
                                    leaf * static_cast<float>(static_cast<int>(rng() % 21) - 10),
                                    leaf * static_cast<float>(static_cast<int>(rng() % 41) - 20)};
                const int side = static_cast<int>(rng() % 3);   // exact, below, above
                float q[3];
                for (int a = 0; a < 3; ++a)
                    q[a] = side == 0 ? f[a] : (side == 1 ? std::nextafter(f[a], -INFINITY)
                                                         : std::nextafter(f[a], INFINITY));
                v.push_back(makePoint(q[0], q[1], q[2], rng));
            }
        testEquivalence("on boundaries (0.1)", makeCloud(v, true), 0.1f);
        testEquivalence("on boundaries (0.3)", makeCloud(v, true), 0.3f);
    }
    {   // NaN and inf points, cloud flagged non-dense as the pipeline's clouds are
        std::vector<PointT> v;
        for (int i = 0; i < 200000; ++i)
        {
            PointT p = makePoint(uni(-4, 4), uni(-4, 4), uni(-4, 4), rng);
            const int bad = static_cast<int>(rng() % 12);
            if (bad == 0) p.x = std::numeric_limits<float>::quiet_NaN();
            else if (bad == 1) p.y = std::numeric_limits<float>::infinity();
            else if (bad == 2) p.z = -std::numeric_limits<float>::infinity();
            v.push_back(p);
        }
        testEquivalence("NaN / inf", makeCloud(v, false), 0.1f);
        std::vector<PointT> allBad(100, makePoint(0, 0, 0, rng));
        for (auto &p : allBad) p.y = std::numeric_limits<float>::quiet_NaN();
        testEquivalence("all non-finite", makeCloud(allBad, false), 0.1f);
    }
    {   // degenerate shapes
        std::vector<PointT> one = {makePoint(1.23f, -4.56f, 7.89f, rng)};
        testEquivalence("single point", makeCloud(one, true), 0.1f);
        std::vector<PointT> same(500, makePoint(-0.05f, 0.05f, 0.0f, rng));
        testEquivalence("identical points", makeCloud(same, true), 0.1f);
        std::vector<PointT> line;   // one cell wide in y and z: the long axis is x
        for (int i = 0; i < 50000; ++i) line.push_back(makePoint(uni(-30, 30), uni(0, 0.09f), uni(-0.09f, 0), rng));
        testEquivalence("thin line", makeCloud(line, true), 0.1f);
        std::vector<PointT> plane;   // flat in z
        for (int i = 0; i < 80000; ++i) plane.push_back(makePoint(uni(-10, 10), uni(-6, 6), 2.0f, rng));
        testEquivalence("flat plane", makeCloud(plane, true), 0.1f);
        std::vector<PointT> box;    // KITTI-like proportions, scaled to a valid single pass
        for (int i = 0; i < 400000; ++i) box.push_back(makePoint(uni(0, 65), uni(0, 4.5f), uni(0, 58), rng));
        testEquivalence("elongated box", makeCloud(box, true), 0.1f);
    }
}

// ---------------------------------------------------------------------------
// (b) overflowing box
// ---------------------------------------------------------------------------

static void testOverflowCloud(const std::string &rawPath)
{
    std::printf("(b) 650 x 45 x 580 m at leaf 0.1 (pcl's grid overflows)\n");
    std::mt19937 rng(777);
    auto uni = [&](float a, float b) { return std::uniform_real_distribution<float>(a, b)(rng); };
    auto clampTo = [](float v, float lo, float hi) { return std::min(std::max(v, lo), hi); };

    // Anchors spread over the whole box, each with a few jittered copies so many
    // voxels hold several points, a share of them on exact voxel boundaries, and the
    // box corners to pin the extent.
    std::vector<PointT> v;
    const float X = 650.f, Y = 45.f, Z = 580.f, leaf = 0.1f;
    std::normal_distribution<float> jit(0.0f, 0.03f);
    for (int i = 0; i < 1000000; ++i)
    {
        float a[3] = {uni(0, X), uni(0, Y), uni(0, Z)};
        if (i % 7 == 0)
            for (float &c : a) c = leaf * std::floor(c / leaf);        // on a boundary
        const int k = 1 + static_cast<int>(rng() % 5);
        for (int j = 0; j < k; ++j)
            v.push_back(makePoint(clampTo(a[0] + jit(rng), 0, X), clampTo(a[1] + jit(rng), 0, Y),
                                  clampTo(a[2] + jit(rng), 0, Z), rng));
    }
    for (int c = 0; c < 8; ++c)
        v.push_back(makePoint((c & 1) ? X : 0.f, (c & 2) ? Y : 0.f, (c & 4) ? Z : 0.f, rng));
    PointCloudT::Ptr cloud = makeCloud(v, true);
    const size_t N = cloud->size();
    std::printf("  synthetic cloud: %zu points\n", N);

    {   // raw copy for the independent numpy count
        std::ofstream f(rawPath, std::ios::binary);
        for (const auto &p : cloud->points) { const float xyz[3] = {p.x, p.y, p.z}; f.write(reinterpret_cast<const char*>(xyz), 12); }
    }

    const VoxelExtent ext = voxelExtent(*cloud, 1.0f / leaf);
    std::printf("  pcl dx,dy,dz = %lld, %lld, %lld  -> %.3e cells (INT32_MAX = 2.147e9)\n",
                static_cast<long long>(ext.d[0]), static_cast<long long>(ext.d[1]),
                static_cast<long long>(ext.d[2]), double(ext.d[0]) * ext.d[1] * ext.d[2]);
    CHECK(ext.overflows, "estimate says the grid overflows");

    // Expected number of voxels, by brute force (sorted keys, no hashing, no pcl).
    std::vector<std::int64_t> keys;
    keys.reserve(N);
    const float inv = 1.0f / leaf;
    for (const auto &p : cloud->points) { const auto c = cellOf(p, inv); keys.push_back(packKey(c[0], c[1], c[2])); }
    std::sort(keys.begin(), keys.end());
    const size_t nCells = static_cast<size_t>(std::unique(keys.begin(), keys.end()) - keys.begin());
    std::printf("  brute-force occupied voxels: %zu (%.3f input points per voxel)\n", nCells, double(N) / nCells);

    PointCloudMapping::Config cfg;
    cfg.enabled = true; cfg.octomapEnabled = false; cfg.offline = true;   // offline w/o spill: no thread
    cfg.resolution = leaf;

    {   // legacy: pcl warns and returns the input untouched
        cfg.voxelSafe = false;
        PointCloudMapping::Impl impl(cfg, System::RGBD);
        impl.mpGlobalCloud = PointCloudT::Ptr(new PointCloudT(*cloud));
        CaptureStderr cap;
        impl.VoxelFilterGlobal();
        const std::string err = cap.finish();
        CHECK(err.find("Integer indices would overflow") != std::string::npos, "legacy: pcl prints its overflow warning");
        CHECK(impl.mpGlobalCloud->size() == N, "legacy: the cloud comes back unfiltered (same point count)");
        bool same = impl.mpGlobalCloud->size() == N;
        for (size_t i = 0; same && i < N; i += 997)
            same = std::memcmp(&impl.mpGlobalCloud->points[i], &cloud->points[i], sizeof(PointT)) == 0;
        CHECK(same, "legacy: the points are the input points, in order");
        CHECK(impl.mnVoxelCalls == 1 && impl.mnVoxelOverflow == 1 && impl.mnVoxelSlabbed == 0 &&
              impl.mnVoxelUnslabbed == 0, "legacy: counters = 1 call, 1 overflow, 0 slabbed");
        CHECK(impl.mvGlobalDepth.size() == N, "legacy: depth array resized as before");
        std::printf("  legacy : %zu -> %zu points (unfiltered), pcl warning %s\n", N,
                    impl.mpGlobalCloud->size(), err.empty() ? "NOT seen" : "seen");
    }
    {   // safe
        cfg.voxelSafe = true;
        PointCloudMapping::Impl impl(cfg, System::RGBD);
        impl.mpGlobalCloud = PointCloudT::Ptr(new PointCloudT(*cloud));
        CaptureStderr cap;
        const auto t0 = std::chrono::steady_clock::now();
        impl.VoxelFilterGlobal();
        const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const std::string err = cap.finish();
        CHECK(err.find("Integer indices would overflow") == std::string::npos, "safe: no pcl overflow warning");
        const PointCloudT &out = *impl.mpGlobalCloud;
        CHECK(out.size() == nCells, "safe: exactly one output point per occupied voxel (matches brute-force count)");
        CHECK(impl.mnVoxelCalls == 1 && impl.mnVoxelOverflow == 1 && impl.mnVoxelSlabbed == 1 &&
              impl.mnVoxelUnslabbed == 0, "safe: counters = 1 call, 1 overflow, 1 slabbed");
        CHECK(impl.mvGlobalDepth.size() == out.size(), "safe: depth array resized as before");

        // At most one point per voxel, and every point sits where the oracle says.
        std::vector<std::int64_t> ok;
        ok.reserve(out.size());
        for (const auto &p : out.points) { const auto c = cellOf(p, inv); ok.push_back(packKey(c[0], c[1], c[2])); }
        std::sort(ok.begin(), ok.end());
        // A centroid one ulp across a cell border could share a key with a neighbour; the
        // exact check is the oracle match below, this is the quick one.
        const bool dupKey = std::adjacent_find(ok.begin(), ok.end()) != ok.end();
        std::printf("  safe   : %zu -> %zu points in %.2f s; output re-binned: %s\n", N, out.size(), sec,
                    dupKey ? "two centroids share a cell key" : "every centroid in its own cell");
        CHECK(!dupKey, "safe: re-binning the output puts every centroid in a distinct voxel");
        const PointCloudT::Ptr orc = oracle(*cloud, leaf);
        const MatchResult m = matchSets(*orc, out, leaf, toleranceFor(*cloud, leaf));
        CHECK(m.ok(), "safe: every centroid and colour matches the oracle, one for one");
        std::printf("  safe vs oracle: %zu cells, unmatched %zu, max centroid dev %.2e of the leaf (float sums at 650 m)\n",
                    orc->size(), m.unmatched, m.maxDev / leaf);
        std::ofstream f(rawPath + ".count");
        f << out.size() << "\n";
    }
    {   // the voxelSafe=1 path with a tiny budget must give the same answer (many slabs)
        PointCloudT out;
        size_t nSlabs = 0;
        const VoxelExtent e2 = voxelExtent(*cloud, 1.0f / leaf);
        const bool ok = voxelFilterSlabs(cloud, leaf, e2, std::int64_t(1) << 24, out, &nSlabs);
        CHECK(ok && out.size() == nCells, "safe: a small slab budget gives the same cell count");
        CHECK(nSlabs > 1000, "safe: the small budget really cut the cloud into many slabs");
        std::printf("  budget 2^24 cells: %zu slabs, %zu points\n", nSlabs, out.size());
    }
}

// ---------------------------------------------------------------------------
// (c) edges, counters, formatting, priority boost
// ---------------------------------------------------------------------------

static void testRefusalBoundary()
{
    std::printf("(c1) boundary of pcl's refusal test and the unslabbable fallback\n");
    std::mt19937 rng(1);
    struct Case { int n; bool refuses; };
    // Two points, leaf 1: pcl's dx = (max-min)+1 per axis. 1290^3 = 2,146,689,000 fits
    // in INT32_MAX; 1291^3 = 2,151,685,171 does not.
    for (Case c : {Case{1290, false}, Case{1291, true}})
    {
        std::vector<PointT> v = {makePoint(0, 0, 0, rng),
                                 makePoint(float(c.n - 1), float(c.n - 1), float(c.n - 1), rng)};
        PointCloudT::Ptr cloud = makeCloud(v, true);
        bool accepted = false;
        CaptureStderr cap;   // pcl prints its overflow warning when it refuses
        PointCloudT ref = singlePass(cloud, 1.0f, &accepted);
        const std::string pclErr = cap.finish();
        CHECK((pclErr.find("Integer indices would overflow") != std::string::npos) == c.refuses,
              "pcl warns exactly when it refuses");
        const VoxelExtent e = voxelExtent(*cloud, 1.0f);
        CHECK(accepted == !c.refuses, "pcl accepts/refuses as expected at the limit");
        CHECK(e.overflows == c.refuses, "estimate agrees with pcl at the limit");
        PointCloudT out;
        size_t n = 0;
        const VoxelPath path = voxelFilterSafe(cloud, 1.0f, kVoxelSlabCells, out, &n);
        CHECK(path == (c.refuses ? VoxelPath::Slabs : VoxelPath::SinglePass), "path chosen at the limit");
        CHECK(out.size() == 2, "both points survive");
        std::printf("  2 points, %d^3 cells: pcl %s, estimate %s, path %s (%zu slabs)\n", c.n,
                    accepted ? "accepts" : "refuses", e.overflows ? "overflows" : "fits",
                    path == VoxelPath::Slabs ? "slabs" : "single pass", n);
    }
    {   // a point 2.2e9 leaf cells away: beyond int32 voxel coordinates
        std::vector<PointT> v = {makePoint(0, 0, 0, rng), makePoint(0, 0, 2.2e9f, rng)};
        PointCloudT::Ptr cloud = makeCloud(v, true);
        PointCloudT out;
        const VoxelPath path = voxelFilterSafe(cloud, 1.0f, kVoxelSlabCells, out);
        CHECK(path == VoxelPath::Unslabbed, "coordinates beyond int32 voxels are left unfiltered");
        PointCloudMapping::Config cfg;
        cfg.enabled = true; cfg.octomapEnabled = false; cfg.offline = true; cfg.resolution = 1.0f;
        for (bool safe : {false, true})
        {
            cfg.voxelSafe = safe;
            PointCloudMapping::Impl impl(cfg, System::RGBD);
            impl.mpGlobalCloud = PointCloudT::Ptr(new PointCloudT(*cloud));
            CaptureStderr cap;
            impl.VoxelFilterGlobal();
            const std::string err = cap.finish();
            CHECK(impl.mpGlobalCloud->size() == 2 && impl.mnVoxelOverflow == 1, "extreme cloud stays as it is, counted as overflow");
            CHECK(impl.mnVoxelUnslabbed == (safe ? 1u : 0u), "unslabbed counter only with voxelSafe");
            if (safe) CHECK(err.find("too large to filter in slabs") != std::string::npos, "safe: one-line warning");
            std::printf("  2.2e9-cell extent, voxelSafe=%d: unfiltered, overflow=%zu unslabbed=%zu\n", safe,
                        size_t(impl.mnVoxelOverflow), size_t(impl.mnVoxelUnslabbed));
        }
    }
    {   // cross-section alone above the budget
        std::vector<PointT> v = {makePoint(0, 0, 0, rng), makePoint(100, 3000, 3000, rng)};
        PointCloudT::Ptr cloud = makeCloud(v, true);
        const VoxelExtent e = voxelExtent(*cloud, 1.0f);
        PointCloudT out;
        CHECK(!voxelFilterSlabs(cloud, 1.0f, e, 1000, out), "budget below one layer: refuses instead of guessing");
        CHECK(out.empty(), "...and leaves `out` untouched");
    }
}

static void testIdenticalToLegacyWhenValid()
{
    std::printf("(c2) voxelSafe=1 equals the legacy call where pcl accepts the grid\n");
    std::mt19937 rng(9);
    auto uni = [&](float a, float b) { return std::uniform_real_distribution<float>(a, b)(rng); };
    std::vector<PointT> v;
    for (int i = 0; i < 300000; ++i) v.push_back(makePoint(uni(-10, 10), uni(-2, 2), uni(0, 12), rng));
    PointCloudMapping::Config cfg;
    cfg.enabled = true; cfg.octomapEnabled = false; cfg.offline = true; cfg.resolution = 0.05f;
    PointCloudT::Ptr a, b;
    size_t callsSafe = 0, ovfSafe = 0, slabSafe = 0;
    for (bool safe : {false, true})
    {
        cfg.voxelSafe = safe;
        PointCloudMapping::Impl impl(cfg, System::RGBD);
        impl.mpGlobalCloud = makeCloud(v, false);
        impl.VoxelFilterGlobal();
        (safe ? b : a) = impl.mpGlobalCloud;
        if (safe) { callsSafe = impl.mnVoxelCalls; ovfSafe = impl.mnVoxelOverflow; slabSafe = impl.mnVoxelSlabbed; }
        else CHECK(impl.mnVoxelCalls == 1 && impl.mnVoxelOverflow == 0, "legacy counters on a valid cloud");
    }
    // PointXYZRGB is 32 bytes with 16 bytes of padding that pcl leaves uninitialised, so compare
    // the fields (x, y, z, rgba), not the struct bytes.
    auto sameFields = [](const PointCloudT &p, const PointCloudT &q) {
        bool same = p.size() == q.size();
        for (size_t i = 0; same && i < p.size(); ++i)
            same = std::memcmp(&p.points[i].x, &q.points[i].x, 12) == 0 && p.points[i].rgba == q.points[i].rgba;
        return same;
    };
    const bool same = sameFields(*a, *b);
    CHECK(same, "identical output (same points, same order, same bits)");
    CHECK(callsSafe == 1 && ovfSafe == 0 && slabSafe == 0, "safe counters on a valid cloud: no overflow, no slabs");

    // The legacy branch against a literal copy of the statements it had before this change.
    PointCloudT::Ptr in = makeCloud(v, false), orig(new PointCloudT());
    {
        pcl::VoxelGrid<PointT> voxel;
        voxel.setLeafSize(cfg.resolution, cfg.resolution, cfg.resolution);
        voxel.setInputCloud(in);
        voxel.filter(*orig);
    }
    const bool sameAsOriginal = sameFields(*orig, *a) && a->height == orig->height && a->width == orig->width &&
                                a->is_dense == orig->is_dense;
    CHECK(sameAsOriginal, "voxelSafe=0 output equals the original VoxelFilterGlobal statements (fields, order, header)");
    std::printf("  %zu points -> %zu; voxelSafe=1 == voxelSafe=0: %s; voxelSafe=0 == original statements: %s\n",
                v.size(), a->size(), same ? "yes" : "NO", sameAsOriginal ? "yes" : "NO");
}

static void testLoadConfig()
{
    std::printf("(c5) Dense.voxelSafe in the settings file\n");
    auto write = [](const char *path, const std::string &body) {
        std::ofstream f(path);
        f << "%YAML:1.0\n---\nDense.enabled: 1\n" << body;
    };
    write("/tmp/vs_cfg_absent.yaml", "");
    write("/tmp/vs_cfg_on.yaml", "Dense.voxelSafe: 1\n");
    write("/tmp/vs_cfg_off.yaml", "Dense.voxelSafe: 0\n");
    write("/tmp/vs_cfg_str.yaml", "Dense.voxelSafe: \"true\"\n");
    CHECK(!PointCloudMapping::LoadConfig("/tmp/vs_cfg_absent.yaml").voxelSafe, "key absent: default off");
    CHECK(PointCloudMapping::LoadConfig("/tmp/vs_cfg_on.yaml").voxelSafe, "Dense.voxelSafe: 1 turns it on");
    CHECK(!PointCloudMapping::LoadConfig("/tmp/vs_cfg_off.yaml").voxelSafe, "Dense.voxelSafe: 0 keeps it off");
    CHECK(PointCloudMapping::LoadConfig("/tmp/vs_cfg_str.yaml").voxelSafe, "Dense.voxelSafe: \"true\" is accepted like the other bool keys");
    CHECK(!PointCloudMapping::Config().voxelSafe, "compiled-in default is off");
    for (const char *s : {"absent", "on", "off", "str"}) std::remove((std::string("/tmp/vs_cfg_") + s + ".yaml").c_str());
}

static void testSummaryAndFormat()
{
    std::printf("(c3) summary line and std::cout formatting\n");
    // The state the examples print their "median tracking time" in.
    std::cout.unsetf(std::ios::floatfield);
    std::cout.precision(6);
    const std::ios_base::fmtflags f0 = std::cout.flags();
    const std::streamsize p0 = std::cout.precision();

    PointCloudMapping::Config cfg;
    cfg.enabled = true; cfg.octomapEnabled = false; cfg.offline = true; cfg.resolution = 1.0f;
    cfg.voxelSafe = true;
    PointCloudMapping pcm(cfg, System::RGBD);
    std::vector<PointT> v;
    std::mt19937 rng(3);
    v.push_back(makePoint(0, 0, 0, rng));
    v.push_back(makePoint(1290, 1290, 1290, rng));   // overflows at leaf 1 (see c1)
    v.push_back(makePoint(0.2f, 0.2f, 0.2f, rng));
    pcm.mpImpl->mpGlobalCloud = makeCloud(v, true);
    pcm.mpImpl->VoxelFilterGlobal();
    pcm.mpImpl->mpGlobalCloud = makeCloud({makePoint(0, 0, 0, rng), makePoint(5, 5, 5, rng)}, true);
    pcm.mpImpl->VoxelFilterGlobal();
    pcm.mpImpl->mvtTotal = {1.0}; pcm.mpImpl->mvtDepth = {1.0}; pcm.mpImpl->mvtVoxel = {1.0};
    pcm.mpImpl->mvtMerge = {1.0}; pcm.mpImpl->mvtOctomap = {1.0};

    const std::string text = captureCout([&] { pcm.PrintTimingSummary(); });
    CHECK(text.find("Global voxel filter (voxelSafe=1): 2 calls, 1 would overflow pcl's int32 grid, 1 handled by slabs") !=
          std::string::npos, "summary line has calls, overflow and slab counts");
    CHECK(text.find("Depth Acquisition : 1.00") != std::string::npos, "existing lines keep their fixed/2 format");
    CHECK(std::cout.flags() == f0 && std::cout.precision() == p0, "PrintTimingSummary restores std::cout flags and precision");
    {
        std::ostringstream os;
        std::streambuf *old = std::cout.rdbuf(os.rdbuf());
        std::cout << 0.0312345678 << "\n";
        std::cout.rdbuf(old);
        CHECK(os.str() == "0.0312346\n", "a value printed afterwards keeps default precision (no rounding to 0.03)");
        std::printf("  after PrintTimingSummary, 0.0312345678 prints as %s", os.str().c_str());
    }
    std::printf("  summary: %s", text.substr(text.find("  Global voxel filter")).c_str());

    {   // never filtered globally (probabilistic merge): the line is still printed, with zeros
        PointCloudMapping::Config c0 = cfg;
        c0.voxelSafe = false;
        PointCloudMapping pcm0(c0, System::RGBD);
        pcm0.mpImpl->mvtTotal = {1.0}; pcm0.mpImpl->mvtDepth = {1.0}; pcm0.mpImpl->mvtVoxel = {1.0};
        pcm0.mpImpl->mvtMerge = {1.0}; pcm0.mpImpl->mvtOctomap = {1.0};
        const std::string t0 = captureCout([&] { pcm0.PrintTimingSummary(); });
        CHECK(t0.find("Global voxel filter (voxelSafe=0): 0 calls, 0 would overflow pcl's int32 grid, 0 handled by slabs, 0.00 s") !=
              std::string::npos, "summary line is printed with zeros when the global filter never ran");
        CHECK(t0.find("left unfiltered") == std::string::npos, "no 'left unfiltered' part without such calls");
    }

    {   // FinalizeAll() (final-pose modes) printed with fixed/2 and never restored it
        PointCloudMapping::Config c2 = cfg;
        c2.voxelSafe = false;
        PointCloudMapping::Impl impl(c2, System::RGBD);   // offline => stores camera clouds
        const std::string t2 = captureCout([&] { impl.FinalizeAll(); });
        CHECK(t2.find("final-pose reconstruction of 0 keyframes took 0.00 s") != std::string::npos, "FinalizeAll still prints its fixed/2 line");
        CHECK(std::cout.flags() == f0 && std::cout.precision() == p0, "FinalizeAll restores std::cout flags and precision");
    }
    {   // Save() with the compression report. Online mode: offline would rebuild (and empty) the cloud.
        PointCloudMapping::Config c3 = cfg;
        c3.offline = false; c3.voxelSafe = false;
        c3.octomapEnabled = true; c3.octomapResolution = 0.5f; c3.compressionReport = true;
        c3.saveDirectory = "/tmp"; c3.savePrefix = "vs_test";
        PointCloudMapping pcm3(c3, System::RGBD);
        pcm3.mpImpl->mpGlobalCloud = makeCloud({makePoint(0, 0, 0, rng), makePoint(5, 5, 5, rng), makePoint(2, 2, 2, rng)}, true);
        pcm3.mpImpl->mpOctree->updateNode(0.f, 0.f, 0.f, true);
        pcm3.mpImpl->mpOctree->updateNode(5.f, 5.f, 5.f, true);
        const std::string t3 = captureCout([&] { pcm3.Save(); });
        CHECK(t3.find("compression report") != std::string::npos, "Save printed the compression report");
        CHECK(std::cout.flags() == f0 && std::cout.precision() == p0, "Save restores std::cout flags and precision");
        for (const char *s : {"/tmp/vs_test_cloud.pcd", "/tmp/vs_test_octomap.ot", "/tmp/vs_test_octomap.bt"}) std::remove(s);
    }
}

static void testPrioBoost()
{
    std::printf("(c4) PrioBoost: detection and the one-line warning\n");
    // What the kernel allows here: SCHED_IDLE, then back to SCHED_OTHER?
    bool canBoost = false;
    std::thread probe([&] {
        sched_param sp; sp.sched_priority = 0;
        if (pthread_setschedparam(pthread_self(), SCHED_IDLE, &sp) == 0)
            canBoost = (pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp) == 0);
    });
    probe.join();

    const size_t before = gnBoostRefused;
    bool active1 = false, active2 = false;
    CaptureStderr cap;
    std::thread dense([&] {
        sched_param sp; sp.sched_priority = 0;
        pthread_setschedparam(pthread_self(), SCHED_IDLE, &sp);
        tlIdleSched = true;
        { PrioBoost b; active1 = b.active; }
        { PrioBoost b; active2 = b.active; }
    });
    dense.join();
    const std::string err = cap.finish();
    const size_t refused = gnBoostRefused - before;
    std::printf("  kernel %s the boost here; PrioBoost.active = %d,%d; refused counter +%zu; warnings printed: %zu\n",
                canBoost ? "allows" : "refuses", active1, active2, refused, countOf(err, "[Dense] WARNING"));
    CHECK(active1 == canBoost && active2 == canBoost, "PrioBoost.active mirrors what the kernel allowed");
    CHECK(refused == (canBoost ? 0u : 2u), "every refused boost is counted");
    CHECK(countOf(err, "[Dense] WARNING") == (canBoost ? 0u : 1u), "exactly one warning, only when the boost fails");
    CHECK(countOf(err, "\n") == (canBoost ? 0u : 1u), "the warning is a single line");
    if (!canBoost)
        std::printf("  warning text: %s", err.c_str());

    {   // a thread that never went idle must not warn or count
        const size_t b2 = gnBoostRefused;
        bool a = true;
        std::thread t([&] { PrioBoost g; a = g.active; });
        t.join();
        CHECK(!a && gnBoostRefused == b2, "a normal-priority thread is untouched");
    }
}

int main(int argc, char **argv)
{
    const std::string rawPath = argc > 1 ? argv[1] : "/tmp/vs_synth_xyz.f32";
    // Every Impl prints a start-up banner on std::cout; keep it out of the report. The tests
    // that look at the output swap their own buffer in (captureCout); flags are not affected.
    std::ostringstream banners;
    std::streambuf *coutOrig = std::cout.rdbuf(banners.rdbuf());
    testSlabEquivalence();
    testOverflowCloud(rawPath);
    testRefusalBoundary();
    testIdenticalToLegacyWhenValid();
    testLoadConfig();
    testSummaryAndFormat();
    testPrioBoost();
    std::cout.rdbuf(coutOrig);
    std::printf("\n%d checks, %d failed\n", gChecks, gFailed);
    return gFailed == 0 ? 0 : 1;
}
