/**
* This file is part of ORB-SLAM3
*
* Dense Reconstruction thread reproducing Hanxiang Zhang, "Dense Reconstruction
* from Visual SLAM with Probabilistic Multi-Sequence Merging", MASc thesis,
* Dalhousie University, 2023. Section and equation numbers refer to it.
*
* Compiled as C++17 (PCL 1.15 requires it) while the rest of the library stays
* C++11; see CMakeLists.txt. Without PCL/octomap the feature degrades to no-ops.
*/

#include "PointCloudMapping.h"

#include <algorithm>
#include <iostream>
#include <string>

// ---------------------------------------------------------------------------
// Settings parsing. Compiled either way so a settings file can always be read.
// ---------------------------------------------------------------------------

namespace ORB_SLAM3
{

static void readFloat(cv::FileStorage &fs, const std::string &key, float &out)
{
    const cv::FileNode n = fs[key];
    if (!n.empty() && n.isReal()) out = static_cast<float>(n.real());
    else if (!n.empty() && n.isInt()) out = static_cast<float>(static_cast<int>(n));
}

static void readInt(cv::FileStorage &fs, const std::string &key, int &out)
{
    const cv::FileNode n = fs[key];
    if (!n.empty() && n.isInt()) out = static_cast<int>(n);
}

static void readBool(cv::FileStorage &fs, const std::string &key, bool &out)
{
    const cv::FileNode n = fs[key];
    if (n.empty()) return;
    if (n.isInt()) out = (static_cast<int>(n) != 0);
    else if (n.isString())
    {
        const std::string s = static_cast<std::string>(n);
        out = (s == "1" || s == "true" || s == "True" || s == "yes");
    }
}

static void readString(cv::FileStorage &fs, const std::string &key, std::string &out)
{
    const cv::FileNode n = fs[key];
    if (!n.empty() && n.isString()) out = static_cast<std::string>(n);
}

PointCloudMapping::Config PointCloudMapping::LoadConfig(const std::string &settingsFile)
{
    Config cfg;
    cv::FileStorage fs(settingsFile, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "[Dense] could not open settings file " << settingsFile
                  << "; dense reconstruction stays disabled." << std::endl;
        return cfg;
    }

    readBool  (fs, "Dense.enabled",            cfg.enabled);
    readFloat (fs, "Dense.resolution",         cfg.resolution);
    readFloat (fs, "Dense.depthMin",           cfg.depthMin);
    readFloat (fs, "Dense.depthMax",           cfg.depthMax);
    readBool  (fs, "Dense.probabilisticMerge", cfg.probabilisticMerge);
    readFloat (fs, "Dense.probMax",            cfg.probMax);
    readFloat (fs, "Dense.mergeDistance",      cfg.mergeDistance);
    readInt   (fs, "Dense.sgbmNumDisparities", cfg.sgbmNumDisparities);
    readInt   (fs, "Dense.sgbmBlockSize",      cfg.sgbmBlockSize);
    readInt   (fs, "Dense.roiMarginX",         cfg.roiMarginX);
    readInt   (fs, "Dense.roiMarginY",         cfg.roiMarginY);
    readBool  (fs, "Dense.colorizeByDepth",    cfg.colorizeByDepth);
    readBool  (fs, "Dense.octomapEnabled",     cfg.octomapEnabled);
    readFloat (fs, "Dense.octomapResolution",  cfg.octomapResolution);
    readBool  (fs, "Dense.octomapRayCast",     cfg.octomapRayCast);
    readString(fs, "Dense.saveDirectory",      cfg.saveDirectory);
    readString(fs, "Dense.savePrefix",         cfg.savePrefix);
    readString(fs, "Dense.loadCloud",          cfg.loadCloud);

    {
        const cv::FileNode n = fs["Dense.solidColor"];
        if (!n.empty() && n.size() == 3)
            cfg.solidColor = cv::Vec3b(static_cast<uchar>(static_cast<int>(n[0])),
                                       static_cast<uchar>(static_cast<int>(n[1])),
                                       static_cast<uchar>(static_cast<int>(n[2])));
    }
    fs.release();

    if (cfg.mergeDistance <= 0.0f) cfg.mergeDistance = cfg.resolution;
    // SGBM demands numDisparities divisible by 16 and an odd, >=1 block size.
    if (cfg.sgbmNumDisparities % 16 != 0)
        cfg.sgbmNumDisparities = std::max(16, (cfg.sgbmNumDisparities / 16) * 16);
    if (cfg.sgbmBlockSize % 2 == 0) cfg.sgbmBlockSize += 1;
    if (cfg.sgbmBlockSize < 1) cfg.sgbmBlockSize = 1;
    // Pmax must stay strictly inside (0.5, 1) or eq. (34)'s k is degenerate.
    cfg.probMax = std::min(std::max(cfg.probMax, 0.51f), 0.99f);
    if (cfg.depthMax <= cfg.depthMin) cfg.depthMax = cfg.depthMin + 1.0f;

    return cfg;
}

} // namespace ORB_SLAM3


#ifndef WITH_DENSE_RECONSTRUCTION
// ---------------------------------------------------------------------------
// Stub build: PCL and/or octomap were not found by CMake.
// ---------------------------------------------------------------------------
namespace ORB_SLAM3
{

struct PointCloudMapping::Impl {};

bool PointCloudMapping::Available() { return false; }

PointCloudMapping::PointCloudMapping(const Config &, int) : mpImpl(nullptr)
{
    std::cerr << "[Dense] this binary was built without PCL/octomap, so dense "
                 "reconstruction is unavailable. Install libpcl-dev and "
                 "liboctomap-dev and rebuild." << std::endl;
}

PointCloudMapping::~PointCloudMapping() {}
void PointCloudMapping::InsertKeyFrameRGBD(KeyFrame *, const cv::Mat &, const cv::Mat &) {}
void PointCloudMapping::InsertKeyFrameStereo(KeyFrame *, const cv::Mat &, const cv::Mat &) {}
void PointCloudMapping::RequestFinish() {}
bool PointCloudMapping::IsFinished() { return true; }
void PointCloudMapping::Save() {}
bool PointCloudMapping::LoadCloud(const std::string &) { return false; }
size_t PointCloudMapping::CloudSize() { return 0; }
void PointCloudMapping::PrintTimingSummary() {}

} // namespace ORB_SLAM3

#else
// ---------------------------------------------------------------------------
// Full build
// ---------------------------------------------------------------------------

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iomanip>
#include <list>
#include <mutex>
#include <numeric>
#include <thread>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <sophus/se3.hpp>

#include "KeyFrame.h"
#include "System.h"

namespace ORB_SLAM3
{

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

/** Rainbow colourisation, sec. 4.2: "red is the farthest and vice versa". */
static cv::Vec3b depthToRainbow(float z, float dmin, float dmax)
{
    const float t = std::min(std::max((z - dmin) / std::max(dmax - dmin, 1e-6f), 0.0f), 1.0f);
    const float hue = (1.0f - t) * 240.0f;   // 240 deg (blue, near) -> 0 deg (red, far)
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(static_cast<uchar>(hue * 0.5f), 255, 255));
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    return bgr.at<cv::Vec3b>(0, 0);
}

struct PointCloudMapping::Impl
{
    struct QueueItem
    {
        KeyFrame *pKF = nullptr;
        cv::Mat   imA;   // colour (RGB-D) or left (stereo)
        cv::Mat   imB;   // depth  (RGB-D) or right (stereo)
    };

    Config mCfg;
    int    mSensor;
    float  mSigmoidK  = 1.0f;   // k,  eq. (34)
    float  mSigmoidD0 = 1.0f;   // d0, eq. (34)

    PointCloudT::Ptr      mpGlobalCloud;
    std::vector<float>    mvGlobalDepth;   // d per point; confidence = Confidence(d)
    octomap::ColorOcTree *mpOctree = nullptr;
    cv::Ptr<cv::StereoSGBM> mpSGBM;

    std::list<QueueItem>    mlQueue;
    std::mutex              mMutexQueue;
    std::condition_variable mQueueUpdated;

    std::mutex mMutexCloud;
    std::mutex mMutexFinish;
    bool mbFinishRequested = false;
    bool mbFinished        = false;

    std::thread mThread;

    std::vector<double> mvtDepth, mvtVoxel, mvtMerge, mvtOctomap, mvtTotal;
    std::mutex mMutexTiming;

    Impl(const Config &cfg, int sensor);
    ~Impl();

    float Confidence(float d) const;
    PointCloudT::Ptr GenerateCloudRGBD(KeyFrame *pKF, const cv::Mat &imColor,
                                       const cv::Mat &imDepth);
    PointCloudT::Ptr GenerateCloudStereo(KeyFrame *pKF, const cv::Mat &imLeft,
                                         const cv::Mat &imRight);
    void MergeProbabilistic(const PointCloudT::Ptr &cloudWorld, const std::vector<float> &depths);
    void MergeAppend(const PointCloudT::Ptr &cloudWorld, const std::vector<float> &depths);
    void VoxelFilterGlobal();
    void InsertIntoOctomap(const PointCloudT::Ptr &cloudWorld, const Sophus::SE3f &Twc);
    void Run();
};

PointCloudMapping::Impl::Impl(const Config &cfg, int sensor)
    : mCfg(cfg), mSensor(sensor)
{
    mpGlobalCloud.reset(new PointCloudT());

    // eq. (34): k = 2*ln(1/Pmax - 1)/(dmin - dmax), d0 = (dmin + dmax)/2.
    // For Pmax > 0.5 both numerator and denominator are negative, so k > 0 and
    // the confidence falls off with depth, as sec. 3.5 requires.
    const float dmin = mCfg.depthMin, dmax = mCfg.depthMax;
    mSigmoidD0 = 0.5f * (dmin + dmax);
    mSigmoidK  = 2.0f * std::log(1.0f / mCfg.probMax - 1.0f) / (dmin - dmax);

    if (mCfg.octomapEnabled)
    {
        mpOctree = new octomap::ColorOcTree(mCfg.octomapResolution);
        mpOctree->setProbHit(0.7);
        mpOctree->setProbMiss(0.4);
        mpOctree->setClampingThresMin(0.12);
        mpOctree->setClampingThresMax(0.97);
    }

    if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
    {
        // Algorithm 2 line 11. SGBM is what the thesis's Disparity() call
        // amounts to in OpenCV terms (sec. 3.3 cites OpenCV for this step).
        const int P1 = 8  * mCfg.sgbmBlockSize * mCfg.sgbmBlockSize;
        const int P2 = 32 * mCfg.sgbmBlockSize * mCfg.sgbmBlockSize;
        mpSGBM = cv::StereoSGBM::create(0, mCfg.sgbmNumDisparities, mCfg.sgbmBlockSize,
                                        P1, P2, 1, 63, 10, 100, 32,
                                        cv::StereoSGBM::MODE_SGBM);
    }

    if (!mCfg.loadCloud.empty())
    {
        PointCloudT::Ptr loaded(new PointCloudT());
        if (pcl::io::loadPCDFile<PointT>(mCfg.loadCloud, *loaded) >= 0)
        {
            mpGlobalCloud = loaded;
            mvGlobalDepth.assign(mpGlobalCloud->size(), mSigmoidD0);
            std::cout << "[Dense] loaded " << mpGlobalCloud->size()
                      << " prior points from " << mCfg.loadCloud << std::endl;
        }
        else
            std::cerr << "[Dense] could not load prior cloud " << mCfg.loadCloud << std::endl;
    }

    std::cout << "[Dense] reconstruction enabled" << std::endl
              << "        voxel resolution   : " << mCfg.resolution << " m" << std::endl
              << "        depth range        : [" << dmin << ", " << dmax << "] m" << std::endl
              << "        probabilistic merge: " << (mCfg.probabilisticMerge ? "on" : "off")
              << " (Pmax=" << mCfg.probMax << ", d=" << mCfg.mergeDistance << " m)" << std::endl
              << "        octomap            : "
              << (mCfg.octomapEnabled ? std::to_string(mCfg.octomapResolution) + " m" : std::string("off"))
              << std::endl;

    mThread = std::thread(&PointCloudMapping::Impl::Run, this);
}

PointCloudMapping::Impl::~Impl()
{
    {
        std::unique_lock<std::mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }
    mQueueUpdated.notify_all();
    if (mThread.joinable()) mThread.join();
    delete mpOctree;
}

float PointCloudMapping::Impl::Confidence(float d) const
{
    // P(d) = 1 - 1/(1 + exp(-k (d - d0))). Passes through (dmin, Pmax) and
    // (dmax, 1 - Pmax), with P(d0) = 0.5.
    const float dc = std::min(std::max(d, mCfg.depthMin), mCfg.depthMax);
    return 1.0f - 1.0f / (1.0f + std::exp(-mSigmoidK * (dc - mSigmoidD0)));
}

// --- Algorithm 1 ------------------------------------------------------------

PointCloudT::Ptr
PointCloudMapping::Impl::GenerateCloudRGBD(KeyFrame *pKF, const cv::Mat &imColor,
                                           const cv::Mat &imDepth)
{
    PointCloudT::Ptr cloud(new PointCloudT());
    cloud->reserve(static_cast<size_t>(imDepth.rows) * imDepth.cols / 4);

    const float fx = pKF->fx, fy = pKF->fy, cx = pKF->cx, cy = pKF->cy;

    // NOTE ON THE THESIS'S INDEXING. Algorithm 1 lines 14-15 read
    //     x <- (row - cx) * z/fx ;  y <- (col - cy) * z/fy
    // driving the horizontal coordinate from the row index. Taken literally that
    // transposes the cloud. The standard pinhole back-projection is used here --
    // column is the horizontal pixel u, row the vertical pixel v -- which is
    // what eq. (29) and Figure 14 actually describe. Everything else follows the
    // algorithm as written.
    for (int v = 0; v < imDepth.rows; ++v)
    {
        const float *rowD = imDepth.ptr<float>(v);
        for (int u = 0; u < imDepth.cols; ++u)
        {
            const float z = rowD[u];                                   // line 13
            if (!std::isfinite(z) || z < mCfg.depthMin || z > mCfg.depthMax) continue;

            PointT p;
            p.x = (static_cast<float>(u) - cx) * z / fx;                // line 14
            p.y = (static_cast<float>(v) - cy) * z / fy;                // line 15
            p.z = z;

            if (imColor.type() == CV_8UC3)                              // lines 16-18
            {
                const cv::Vec3b &bgr = imColor.at<cv::Vec3b>(v, u);
                p.b = bgr[0]; p.g = bgr[1]; p.r = bgr[2];
            }
            else if (imColor.type() == CV_8UC1)
            {
                const uchar g = imColor.at<uchar>(v, u);
                p.b = p.g = p.r = g;
            }
            else { p.b = p.g = p.r = 200; }

            cloud->push_back(p);                                        // line 19
        }
    }
    cloud->width  = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

// --- Algorithm 2 ------------------------------------------------------------

PointCloudT::Ptr
PointCloudMapping::Impl::GenerateCloudStereo(KeyFrame *pKF, const cv::Mat &imLeft,
                                             const cv::Mat &imRight)
{
    PointCloudT::Ptr cloud(new PointCloudT());

    cv::Mat grayL = imLeft, grayR = imRight;
    if (grayL.channels() == 3) cv::cvtColor(imLeft,  grayL, cv::COLOR_BGR2GRAY);
    if (grayR.channels() == 3) cv::cvtColor(imRight, grayR, cv::COLOR_BGR2GRAY);
    if (grayL.size() != grayR.size() || grayL.empty()) return cloud;

    cv::Mat disp16;
    mpSGBM->compute(grayL, grayR, disp16);                              // line 11
    cv::Mat disp;
    disp16.convertTo(disp, CV_32F, 1.0 / 16.0);  // SGBM packs 4 fractional bits

    const float fx = pKF->fx, fy = pKF->fy, cx = pKF->cx, cy = pKF->cy;
    const float bf = pKF->mbf;                   // baseline * fx, so z = bf/d

    // line 13: region of interest. ORB-SLAM3 already rectified the pair in
    // System::TrackStereo, so the ROI here only trims the invalid border left by
    // rectification and by the SGBM search window.
    const int uMin = std::max(mCfg.roiMarginX, mCfg.sgbmNumDisparities);
    const int uMax = disp.cols - std::max(mCfg.roiMarginX, 1);
    const int vMin = std::max(mCfg.roiMarginY, 1);
    const int vMax = disp.rows - std::max(mCfg.roiMarginY, 1);
    if (uMin >= uMax || vMin >= vMax) return cloud;

    cloud->reserve(static_cast<size_t>(vMax - vMin) * static_cast<size_t>(uMax - uMin) / 4);

    for (int v = vMin; v < vMax; ++v)
    {
        const float *rowD = disp.ptr<float>(v);
        for (int u = uMin; u < uMax; ++u)
        {
            const float d = rowD[u];
            if (!std::isfinite(d) || d <= 0.0f) continue;

            const float z = bf / d;                                     // line 17
            if (z < mCfg.depthMin || z > mCfg.depthMax) continue;

            PointT p;
            p.x = (static_cast<float>(u) - cx) * z / fx;                // line 18
            p.y = (static_cast<float>(v) - cy) * z / fy;                // line 19
            p.z = z;

            const cv::Vec3b bgr = mCfg.colorizeByDepth                   // lines 20-22
                                ? depthToRainbow(z, mCfg.depthMin, mCfg.depthMax)
                                : mCfg.solidColor;
            p.b = bgr[0]; p.g = bgr[1]; p.r = bgr[2];

            cloud->push_back(p);                                        // line 23
        }
    }
    cloud->width  = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

// --- eq. (35) ---------------------------------------------------------------

void PointCloudMapping::Impl::MergeProbabilistic(const PointCloudT::Ptr &cloudWorld,
                                                 const std::vector<float> &depths)
{
    std::unique_lock<std::mutex> lock(mMutexCloud);

    if (mpGlobalCloud->empty())
    {
        *mpGlobalCloud = *cloudWorld;
        mvGlobalDepth = depths;
        return;
    }

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(mpGlobalCloud);

    const float r = mCfg.mergeDistance;
    std::vector<bool> removed(mpGlobalCloud->size(), false);
    PointCloudT appended;
    std::vector<float> appendedDepth;

    std::vector<int>   idx;
    std::vector<float> sqd;

    for (size_t i = 0; i < cloudWorld->size(); ++i)
    {
        PointT c = cloudWorld->points[i];
        float  d = depths[i];

        // "iterated as many times as necessary until no map point can be found
        // within the minimum spacing" (sec. 3.5). The tree is a snapshot of the
        // global cloud taken before this keyframe, so points appended from this
        // same keyframe are not re-searched -- the per-keyframe voxel filter has
        // already spaced those at least `resolution` apart.
        for (int iter = 0; iter < 8; ++iter)
        {
            idx.clear(); sqd.clear();
            if (kdtree.radiusSearch(c, r, idx, sqd) <= 0) break;

            int hit = -1;
            for (size_t k = 0; k < idx.size(); ++k)
                if (!removed[idx[k]]) { hit = idx[k]; break; }
            if (hit < 0) break;

            const float P1 = Confidence(d);
            const float P2 = Confidence(mvGlobalDepth[hit]);
            const float wsum = P1 + P2;
            const float w1 = (wsum > 1e-9f) ? P1 / wsum : 0.5f;
            const float w2 = 1.0f - w1;

            const PointT &c2 = mpGlobalCloud->points[hit];
            PointT c3;
            c3.x = w1 * c.x + w2 * c2.x;                                // eq. (35)
            c3.y = w1 * c.y + w2 * c2.y;
            c3.z = w1 * c.z + w2 * c2.z;
            c3.r = static_cast<uint8_t>(w1 * c.r + w2 * c2.r);
            c3.g = static_cast<uint8_t>(w1 * c.g + w2 * c2.g);
            c3.b = static_cast<uint8_t>(w1 * c.b + w2 * c2.b);

            removed[hit] = true;
            // S(c) = [x, y, z, d], so the depth component is averaged too and
            // P3 then follows from eq. (34) applied to the merged depth.
            d = w1 * d + w2 * mvGlobalDepth[hit];
            c = c3;
        }

        appended.push_back(c);
        appendedDepth.push_back(d);
    }

    PointCloudT::Ptr out(new PointCloudT());
    std::vector<float> outDepth;
    out->reserve(mpGlobalCloud->size() + appended.size());
    outDepth.reserve(mvGlobalDepth.size() + appendedDepth.size());
    for (size_t i = 0; i < mpGlobalCloud->size(); ++i)
        if (!removed[i]) { out->push_back(mpGlobalCloud->points[i]); outDepth.push_back(mvGlobalDepth[i]); }
    for (size_t i = 0; i < appended.size(); ++i)
    { out->push_back(appended.points[i]); outDepth.push_back(appendedDepth[i]); }

    out->width  = static_cast<uint32_t>(out->size());
    out->height = 1;
    out->is_dense = false;
    mpGlobalCloud = out;
    mvGlobalDepth = std::move(outDepth);
}

void PointCloudMapping::Impl::MergeAppend(const PointCloudT::Ptr &cloudWorld,
                                          const std::vector<float> &depths)
{
    std::unique_lock<std::mutex> lock(mMutexCloud);
    *mpGlobalCloud += *cloudWorld;
    mvGlobalDepth.insert(mvGlobalDepth.end(), depths.begin(), depths.end());
}

void PointCloudMapping::Impl::VoxelFilterGlobal()
{
    std::unique_lock<std::mutex> lock(mMutexCloud);
    if (mpGlobalCloud->empty()) return;

    PointCloudT::Ptr filtered(new PointCloudT());
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(mCfg.resolution, mCfg.resolution, mCfg.resolution);
    voxel.setInputCloud(mpGlobalCloud);
    voxel.filter(*filtered);
    mpGlobalCloud = filtered;
    // Only reached when the probabilistic merge is off, in which case the depth
    // array is never read; reset it to d0 to keep the sizes consistent.
    mvGlobalDepth.assign(mpGlobalCloud->size(), mSigmoidD0);
}

// --- sec. 3.4 ---------------------------------------------------------------

void PointCloudMapping::Impl::InsertIntoOctomap(const PointCloudT::Ptr &cloudWorld,
                                                const Sophus::SE3f &Twc)
{
    if (!mpOctree) return;

    if (mCfg.octomapRayCast)
    {
        // Free space is carved along every ray, so eq. (31)-(33)'s log-odds
        // update can also decrease. Costs one node per voxel per ray.
        octomap::Pointcloud opc;
        opc.reserve(cloudWorld->size());
        for (const auto &p : cloudWorld->points) opc.push_back(p.x, p.y, p.z);
        const Eigen::Vector3f t = Twc.translation();
        mpOctree->insertPointCloud(opc, octomap::point3d(t.x(), t.y(), t.z()));
    }
    else
    {
        // Occupied endpoints only. This is what reproduces the compression
        // ratios of Table IX.
        for (const auto &p : cloudWorld->points)
            mpOctree->updateNode(p.x, p.y, p.z, true, true /*lazy_eval*/);
    }

    for (const auto &p : cloudWorld->points)
        mpOctree->integrateNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);
}

// --- consumer thread --------------------------------------------------------

void PointCloudMapping::Impl::Run()
{
    using clock = std::chrono::steady_clock;
    int sinceFilter = 0;

    while (true)
    {
        QueueItem item;
        {
            std::unique_lock<std::mutex> lock(mMutexQueue);
            mQueueUpdated.wait_for(lock, std::chrono::milliseconds(50),
                                   [this] { return !mlQueue.empty(); });
            if (mlQueue.empty())
            {
                std::unique_lock<std::mutex> lf(mMutexFinish);
                if (mbFinishRequested) break;
                continue;
            }
            item = std::move(mlQueue.front());
            mlQueue.pop_front();
        }

        if (item.pKF == nullptr || item.pKF->isBad()) continue;

        const auto t0 = clock::now();

        PointCloudT::Ptr cloudCam =
            (mSensor == System::RGBD || mSensor == System::IMU_RGBD)
                ? GenerateCloudRGBD(item.pKF, item.imA, item.imB)
                : GenerateCloudStereo(item.pKF, item.imA, item.imB);

        const auto t1 = clock::now();

        // sec. 3.2: voxel filter first, "more than 90% of the voxels are
        // removed". Filtering in the camera frame lets eq. (34)'s d be read back
        // exactly as the surviving z.
        PointCloudT::Ptr cloudCamF(new PointCloudT());
        if (!cloudCam->empty())
        {
            pcl::VoxelGrid<PointT> voxel;
            voxel.setLeafSize(mCfg.resolution, mCfg.resolution, mCfg.resolution);
            voxel.setInputCloud(cloudCam);
            voxel.filter(*cloudCamF);
        }
        std::vector<float> depths(cloudCamF->size());
        for (size_t i = 0; i < cloudCamF->size(); ++i) depths[i] = cloudCamF->points[i].z;

        const auto t2 = clock::now();

        // Algorithm 1 line 21 / Algorithm 2 line 25: camera frame -> world. The
        // pose is read now, so it carries whatever the backend has optimised so
        // far for this keyframe.
        const Sophus::SE3f Twc = item.pKF->GetPoseInverse();
        PointCloudT::Ptr cloudWorld(new PointCloudT());
        if (!cloudCamF->empty())
            pcl::transformPointCloud(*cloudCamF, *cloudWorld, Twc.matrix());

        if (!cloudWorld->empty())
        {
            if (mCfg.probabilisticMerge) MergeProbabilistic(cloudWorld, depths);
            else                         MergeAppend(cloudWorld, depths);
        }

        const auto t3 = clock::now();

        if (mCfg.octomapEnabled && !cloudWorld->empty()) InsertIntoOctomap(cloudWorld, Twc);

        const auto t4 = clock::now();

        // The probabilistic merge already enforces the minimum spacing (sec. 3.5
        // ends by noting redundant points are removed), so the global filter is
        // only needed on the plain-append path.
        if (!mCfg.probabilisticMerge && ++sinceFilter >= 10)
        {
            VoxelFilterGlobal();
            sinceFilter = 0;
        }

        auto ms = [](clock::time_point a, clock::time_point b) {
            return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(b - a).count();
        };
        {
            std::unique_lock<std::mutex> lt(mMutexTiming);
            mvtDepth.push_back(ms(t0, t1));
            mvtVoxel.push_back(ms(t1, t2));
            mvtMerge.push_back(ms(t2, t3));
            mvtOctomap.push_back(ms(t3, t4));
            mvtTotal.push_back(ms(t0, t4));
        }
    }

    {
        std::unique_lock<std::mutex> lock(mMutexFinish);
        mbFinished = true;
    }
}

// --- public facade ----------------------------------------------------------

bool PointCloudMapping::Available() { return true; }

PointCloudMapping::PointCloudMapping(const Config &cfg, int sensor)
    : mpImpl(new Impl(cfg, sensor)) {}

PointCloudMapping::~PointCloudMapping() { delete mpImpl; }

void PointCloudMapping::InsertKeyFrameRGBD(KeyFrame *pKF, const cv::Mat &imColor,
                                           const cv::Mat &imDepth)
{
    if (!mpImpl || !mpImpl->mCfg.enabled) return;
    if (pKF == nullptr || imColor.empty() || imDepth.empty()) return;
    Impl::QueueItem item;
    item.pKF = pKF;
    item.imA = imColor.clone();
    item.imB = imDepth.clone();
    {
        std::unique_lock<std::mutex> lock(mpImpl->mMutexQueue);
        mpImpl->mlQueue.push_back(std::move(item));
    }
    mpImpl->mQueueUpdated.notify_one();
}

void PointCloudMapping::InsertKeyFrameStereo(KeyFrame *pKF, const cv::Mat &imLeft,
                                             const cv::Mat &imRight)
{
    if (!mpImpl || !mpImpl->mCfg.enabled) return;
    if (pKF == nullptr || imLeft.empty() || imRight.empty()) return;
    Impl::QueueItem item;
    item.pKF = pKF;
    item.imA = imLeft.clone();
    item.imB = imRight.clone();
    {
        std::unique_lock<std::mutex> lock(mpImpl->mMutexQueue);
        mpImpl->mlQueue.push_back(std::move(item));
    }
    mpImpl->mQueueUpdated.notify_one();
}

void PointCloudMapping::RequestFinish()
{
    if (!mpImpl) return;
    {
        std::unique_lock<std::mutex> lock(mpImpl->mMutexFinish);
        mpImpl->mbFinishRequested = true;
    }
    mpImpl->mQueueUpdated.notify_all();
}

bool PointCloudMapping::IsFinished()
{
    if (!mpImpl) return true;
    std::unique_lock<std::mutex> lock(mpImpl->mMutexFinish);
    return mpImpl->mbFinished;
}

size_t PointCloudMapping::CloudSize()
{
    if (!mpImpl) return 0;
    std::unique_lock<std::mutex> lock(mpImpl->mMutexCloud);
    return mpImpl->mpGlobalCloud ? mpImpl->mpGlobalCloud->size() : 0;
}

void PointCloudMapping::Save()
{
    if (!mpImpl || !mpImpl->mCfg.enabled) return;

    std::unique_lock<std::mutex> lock(mpImpl->mMutexCloud);
    const std::string base = mpImpl->mCfg.saveDirectory + "/" + mpImpl->mCfg.savePrefix;

    if (mpImpl->mpGlobalCloud && !mpImpl->mpGlobalCloud->empty())
    {
        const std::string pcd = base + "_cloud.pcd";
        if (pcl::io::savePCDFileBinary(pcd, *mpImpl->mpGlobalCloud) == 0)
            std::cout << "[Dense] saved " << mpImpl->mpGlobalCloud->size()
                      << " points to " << pcd << std::endl;
        else
            std::cerr << "[Dense] failed to write " << pcd << std::endl;
    }
    else
        std::cerr << "[Dense] global cloud is empty, nothing written." << std::endl;

    if (mpImpl->mpOctree)
    {
        // sec. 3.4: "all eight child nodes under a parent node are pruned if they
        // are assigned with same states". updateInnerOccupancy propagates the
        // children's values up (needed after lazy updateNode calls); prune then
        // collapses the uniform groups, which is where the compression comes from.
        mpImpl->mpOctree->updateInnerOccupancy();
        mpImpl->mpOctree->prune();
        const std::string ot = base + "_octomap.ot";
        if (mpImpl->mpOctree->write(ot))
            std::cout << "[Dense] saved octomap (" << mpImpl->mpOctree->size()
                      << " nodes) to " << ot << std::endl;
        else
            std::cerr << "[Dense] failed to write " << ot << std::endl;
    }
}

bool PointCloudMapping::LoadCloud(const std::string &pcdFile)
{
    if (!mpImpl) return false;
    PointCloudT::Ptr loaded(new PointCloudT());
    if (pcl::io::loadPCDFile<PointT>(pcdFile, *loaded) < 0)
    {
        std::cerr << "[Dense] could not load " << pcdFile << std::endl;
        return false;
    }
    std::unique_lock<std::mutex> lock(mpImpl->mMutexCloud);
    mpImpl->mpGlobalCloud = loaded;
    // Prior points carry no recorded depth, so seed them at d0 (confidence 0.5):
    // a new observation is then weighted purely by its own distance.
    mpImpl->mvGlobalDepth.assign(mpImpl->mpGlobalCloud->size(), mpImpl->mSigmoidD0);
    std::cout << "[Dense] loaded " << mpImpl->mpGlobalCloud->size()
              << " prior points from " << pcdFile << std::endl;
    return true;
}

void PointCloudMapping::PrintTimingSummary()
{
    if (!mpImpl) return;
    std::unique_lock<std::mutex> lock(mpImpl->mMutexTiming);
    if (mpImpl->mvtTotal.empty()) return;

    auto mean = [](const std::vector<double> &v) {
        return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    };

    // Mirrors the Dense Reconstruction block of Table X.
    std::cout << std::endl
              << "Dense Reconstruction timing over " << mpImpl->mvtTotal.size()
              << " keyframes (mean, ms)" << std::endl
              << std::fixed << std::setprecision(2)
              << "  Depth Acquisition : " << mean(mpImpl->mvtDepth)   << std::endl
              << "  Voxel Filtering   : " << mean(mpImpl->mvtVoxel)   << std::endl
              << "  Map Update        : " << mean(mpImpl->mvtMerge)   << std::endl
              << "  Octomap Conversion: " << mean(mpImpl->mvtOctomap) << std::endl
              << "  Total             : " << mean(mpImpl->mvtTotal)   << std::endl;
}

} // namespace ORB_SLAM3

#endif // WITH_DENSE_RECONSTRUCTION
