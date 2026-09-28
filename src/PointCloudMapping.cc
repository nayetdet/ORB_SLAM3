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
#include <atomic>
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

    readInt   (fs, "Dense.queueLimit",         cfg.queueLimit);
    readBool  (fs, "Dense.lowPriority",        cfg.lowPriority);
    {
        std::string mode;
        readString(fs, "Dense.mode", mode);
        if (mode == "offline") cfg.offline = true;
        else if (!mode.empty() && mode != "online")
            std::cerr << "[Dense] unknown Dense.mode '" << mode << "', using online." << std::endl;
    }
    readString(fs, "Dense.offlineSpillDir",    cfg.offlineSpillDir);
    readBool  (fs, "Dense.reprojectOptimized", cfg.reprojectOptimized);
    readBool  (fs, "Dense.octomapAtEnd",       cfg.octomapAtEnd);
    readFloat (fs, "Dense.octomapMaxRange",    cfg.octomapMaxRange);
    readBool  (fs, "Dense.octomapDiscretize",  cfg.octomapDiscretize);
    readBool  (fs, "Dense.outlierRemoval",     cfg.outlierRemoval);
    readInt   (fs, "Dense.outlierMeanK",       cfg.outlierMeanK);
    readFloat (fs, "Dense.outlierStddev",      cfg.outlierStddev);
    readBool  (fs, "Dense.wlsFilter",          cfg.wlsFilter);
    {
        float v;
        v = static_cast<float>(cfg.wlsLambda);     readFloat(fs, "Dense.wlsLambda", v);     cfg.wlsLambda = v;
        v = static_cast<float>(cfg.wlsSigmaColor); readFloat(fs, "Dense.wlsSigmaColor", v); cfg.wlsSigmaColor = v;
    }
    readBool  (fs, "Dense.octomapWriteBt",     cfg.octomapWriteBt);
    readBool  (fs, "Dense.compressionReport",  cfg.compressionReport);

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
    if (cfg.queueLimit < 0) cfg.queueLimit = 0;
    if (cfg.outlierMeanK < 1) cfg.outlierMeanK = 1;

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
void PointCloudMapping::FinalizeOffline() {}
bool PointCloudMapping::NeedsFinalPoses() const { return false; }
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
#include <set>
#include <thread>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#ifdef WITH_XIMGPROC
#include <opencv2/ximgproc.hpp>
#endif

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <sophus/se3.hpp>

#include "KeyFrame.h"
#include "Map.h"
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

// True only on the dense thread when it runs at SCHED_IDLE.
static thread_local bool tlIdleSched = false;

/** A2 priority-inversion guard: while the SCHED_IDLE dense thread holds a mutex
 *  that another thread (tracking, viewer, Save) may wait on, run it at normal
 *  priority so it cannot be starved while holding the lock. */
struct PrioBoost
{
    bool active = false;
    PrioBoost()
    {
        if (!tlIdleSched) return;
        sched_param sp;
        sp.sched_priority = 0;
        active = (pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp) == 0);
    }
    ~PrioBoost()
    {
        if (!active) return;
        sched_param sp;
        sp.sched_priority = 0;
        pthread_setschedparam(pthread_self(), SCHED_IDLE, &sp);
    }
};

struct PointCloudMapping::Impl
{
    struct QueueItem
    {
        KeyFrame *pKF = nullptr;
        cv::Mat   imA;   // colour (RGB-D) or left (stereo)
        cv::Mat   imB;   // depth  (RGB-D) or right (stereo)
        std::string fA, fB;   // offline spill files; images live here when non-empty
    };

    /** Voxel-filtered keyframe cloud kept in the CAMERA frame (B1/A5). */
    struct StoredCloud
    {
        KeyFrame *pKF = nullptr;
        PointCloudT::Ptr cloud;
        std::vector<float> depths;
    };

    Config mCfg;
    int    mSensor;
    float  mSigmoidK  = 1.0f;   // k,  eq. (34)
    float  mSigmoidD0 = 1.0f;   // d0, eq. (34)

    PointCloudT::Ptr      mpGlobalCloud;
    std::vector<float>    mvGlobalDepth;   // d per point; confidence = Confidence(d)
    octomap::ColorOcTree *mpOctree = nullptr;
    cv::Ptr<cv::StereoSGBM> mpSGBM;
#ifdef WITH_XIMGPROC
    cv::Ptr<cv::StereoMatcher> mpRightMatcher;              // B4
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> mpWLS;
#endif

    // Derived mode flags (see Config). Offline and reprojectOptimized both
    // rebuild the world cloud from the stored camera-frame clouds at the end.
    bool mbOffline     = false;
    bool mbReproject   = false;   // rebuild world cloud with final poses
    bool mbOctoAtEnd   = false;   // build octomap with final poses at the end
    bool mbStoreCam    = false;   // keep camera-frame clouds in mvStored

    std::vector<StoredCloud> mvStored;
    std::mutex               mMutexStored;
    std::vector<QueueItem>   mvOffline;    // offline: raw keyframes, guarded by mMutexQueue
    int                      mnSpill = 0;
    std::mutex               mMutexFinalize;
    std::atomic<double>      mFinalizeSeconds{0.0};
    std::atomic<size_t>      mnDropped{0};    // A1
    std::atomic<size_t>      mnRescued{0};    // culled keyframes recovered via the spanning tree
    std::atomic<size_t>      mnSkippedNoPose{0};   // dropped: no live ancestor
    std::atomic<size_t>      mnEmptyCloud{0};      // dropped: keyframe gave an empty cloud
    std::atomic<bool>        mbFinalDone{false};   // final-pose rebuild is up to date

    PointCloudT::Ptr   mpPriorCloud;          // cloud loaded before the run, if any
    std::vector<float> mvPriorDepth;

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

    void Enqueue(QueueItem &&item);
    void PrepareCloud(KeyFrame *pKF, const cv::Mat &imA, const cv::Mat &imB,
                      PointCloudT::Ptr &out, std::vector<float> &depths,
                      double &msDepth, double &msVoxel);
    void IntegrateFinal(const StoredCloud &sc, int &sinceFilter);
    void FinalizeAll();
    void ResetOctree();
    void ApplyLowPriority();
    bool SpillItem(QueueItem &item);
    bool UnspillItem(QueueItem &item);
};

/** Pose of a keyframe as of now. A keyframe culled during the run keeps a stale
 *  pose, so follow the spanning tree (mTcp, set in SetBadFlag) to the nearest
 *  live ancestor, which the backend keeps optimising. */
static bool finalTwc(KeyFrame *pKF, Sophus::SE3f &Twc, bool *pRescued = nullptr)
{
    Sophus::SE3f rel;   // Tcw(pKF) = rel * Tcw(cur)
    KeyFrame *cur = pKF;
    if (pRescued) *pRescued = pKF->isBad();
    for (int guard = 0; cur->isBad(); ++guard)
    {
        KeyFrame *par = cur->GetParent();
        if (par == nullptr || guard > 10000) return false;
        rel = rel * cur->mTcp;
        cur = par;
    }
    Twc = (rel * cur->GetPose()).inverse();
    return true;
}

static bool writeMat(const std::string &path, const cv::Mat &m)
{
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    const cv::Mat c = m.isContinuous() ? m : m.clone();
    const int hdr[3] = {c.rows, c.cols, c.type()};
    f.write(reinterpret_cast<const char*>(hdr), sizeof(hdr));
    f.write(reinterpret_cast<const char*>(c.data), static_cast<std::streamsize>(c.total() * c.elemSize()));
    return static_cast<bool>(f);
}

static bool readMat(const std::string &path, cv::Mat &m)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    int hdr[3];
    f.read(reinterpret_cast<char*>(hdr), sizeof(hdr));
    if (!f) return false;
    m.create(hdr[0], hdr[1], hdr[2]);
    f.read(reinterpret_cast<char*>(m.data), static_cast<std::streamsize>(m.total() * m.elemSize()));
    return static_cast<bool>(f);
}

static std::streamoff fileSize(const std::string &path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    return f ? static_cast<std::streamoff>(f.tellg()) : -1;
}

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

    mbOffline   = mCfg.offline;
    mbReproject = mCfg.reprojectOptimized || mCfg.offline;
    mbOctoAtEnd = mCfg.octomapEnabled && (mCfg.octomapAtEnd || mbReproject);
    mbStoreCam  = mbReproject || mbOctoAtEnd;

    ResetOctree();

    if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
    {
        // Algorithm 2 line 11. SGBM is what the thesis's Disparity() call
        // amounts to in OpenCV terms (sec. 3.3 cites OpenCV for this step).
        const int P1 = 8  * mCfg.sgbmBlockSize * mCfg.sgbmBlockSize;
        const int P2 = 32 * mCfg.sgbmBlockSize * mCfg.sgbmBlockSize;
        mpSGBM = cv::StereoSGBM::create(0, mCfg.sgbmNumDisparities, mCfg.sgbmBlockSize,
                                        P1, P2, 1, 63, 10, 100, 32,
                                        cv::StereoSGBM::MODE_SGBM);
        if (mCfg.wlsFilter)
        {
#ifdef WITH_XIMGPROC
            mpRightMatcher = cv::ximgproc::createRightMatcher(mpSGBM);
            mpWLS = cv::ximgproc::createDisparityWLSFilter(mpSGBM);
            mpWLS->setLambda(mCfg.wlsLambda);
            mpWLS->setSigmaColor(mCfg.wlsSigmaColor);
#else
            std::cerr << "[Dense] Dense.wlsFilter requested but this build has no "
                         "opencv_ximgproc (opencv_contrib); WLS disabled." << std::endl;
#endif
        }
    }

    if (!mCfg.loadCloud.empty())
    {
        PointCloudT::Ptr loaded(new PointCloudT());
        if (pcl::io::loadPCDFile<PointT>(mCfg.loadCloud, *loaded) >= 0)
        {
            mpGlobalCloud = loaded;
            mvGlobalDepth.assign(mpGlobalCloud->size(), mSigmoidD0);
            mpPriorCloud.reset(new PointCloudT(*loaded));
            mvPriorDepth = mvGlobalDepth;
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
    if (mCfg.queueLimit > 0 || mCfg.lowPriority || mbOffline || mbReproject || mbOctoAtEnd ||
        mCfg.outlierRemoval || mCfg.wlsFilter)
        std::cout << "        extensions         : queueLimit=" << mCfg.queueLimit
                  << " lowPriority=" << mCfg.lowPriority
                  << " mode=" << (mbOffline ? "offline" : "online")
                  << " reproject=" << mbReproject
                  << " octomapAtEnd=" << mbOctoAtEnd
                  << " outlierRemoval=" << mCfg.outlierRemoval
                  << " wls=" << mCfg.wlsFilter << std::endl;

    if (mbOffline && mCfg.offlineSpillDir.empty())
    {
        // Nothing to consume during the run: keyframes are only stored, and
        // FinalizeOffline() builds everything afterwards.
        mbFinished = true;
    }
    else if (mbOffline)
    {
        // Spill mode: a worker thread writes the images to disk so the tracking
        // thread never waits on file I/O (see Run()).
        std::filesystem::create_directories(mCfg.offlineSpillDir);
        mThread = std::thread(&PointCloudMapping::Impl::Run, this);
    }
    else
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
    for (auto &it : mvOffline)   // spill files never consumed
    {
        if (!it.fA.empty()) std::remove(it.fA.c_str());
        if (!it.fB.empty()) std::remove(it.fB.c_str());
    }
    delete mpOctree;
}

void PointCloudMapping::Impl::ResetOctree()
{
    delete mpOctree;
    mpOctree = nullptr;
    if (mCfg.octomapEnabled)
    {
        mpOctree = new octomap::ColorOcTree(mCfg.octomapResolution);
        mpOctree->setProbHit(0.7);
        mpOctree->setProbMiss(0.4);
        mpOctree->setClampingThresMin(0.12);
        mpOctree->setClampingThresMax(0.97);
    }
}

void PointCloudMapping::Impl::ApplyLowPriority()
{
    // A2. SCHED_IDLE needs no privileges on Linux; if it is refused, fall back
    // to nice +10 on this thread only (nice is per-thread on Linux).
    sched_param sp;
    sp.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_IDLE, &sp) == 0) { tlIdleSched = true; return; }
    if (setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 10) != 0)
        std::cerr << "[Dense] could not lower the dense thread priority." << std::endl;
}

bool PointCloudMapping::Impl::SpillItem(QueueItem &item)
{
    const std::string base = mCfg.offlineSpillDir + "/orbslam3_dense_" +
                             std::to_string(static_cast<long>(getpid())) + "_" +
                             std::to_string(mnSpill++);
    item.fA = base + "_A.bin";
    item.fB = base + "_B.bin";
    if (writeMat(item.fA, item.imA) && writeMat(item.fB, item.imB))
    {
        item.imA.release();
        item.imB.release();
        return true;
    }
    std::remove(item.fA.c_str());
    std::remove(item.fB.c_str());
    item.fA.clear();
    item.fB.clear();
    return false;   // keep the images in RAM
}

bool PointCloudMapping::Impl::UnspillItem(QueueItem &item)
{
    if (item.fA.empty()) return !item.imA.empty();
    const bool ok = readMat(item.fA, item.imA) && readMat(item.fB, item.imB);
    std::remove(item.fA.c_str());
    std::remove(item.fB.c_str());
    item.fA.clear();
    item.fB.clear();
    return ok;
}

void PointCloudMapping::Impl::Enqueue(QueueItem &&item)
{
    if (mbOffline)
    {
        if (!mCfg.offlineSpillDir.empty())
        {
            // Hand the item to the worker thread; never do disk I/O on the tracking thread.
            {
                std::unique_lock<std::mutex> lock(mMutexQueue);
                mlQueue.push_back(std::move(item));
            }
            mQueueUpdated.notify_one();
            return;
        }
        std::unique_lock<std::mutex> lock(mMutexQueue);
        mvOffline.push_back(std::move(item));
        mbFinalDone = false;
        return;
    }
    mbFinalDone = false;
    {
        std::unique_lock<std::mutex> lock(mMutexQueue);
        mlQueue.push_back(std::move(item));
        // A1: bounded queue, drop the oldest so the newest keyframe always gets in.
        if (mCfg.queueLimit > 0)
            while (mlQueue.size() > static_cast<size_t>(mCfg.queueLimit))
            {
                mlQueue.pop_front();
                ++mnDropped;
            }
    }
    mQueueUpdated.notify_one();
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
#ifdef WITH_XIMGPROC
    if (mpWLS)
    {
        cv::Mat dispR16, filtered;
        mpSGBM->compute(grayL, grayR, disp16);                          // line 11
        mpRightMatcher->compute(grayR, grayL, dispR16);
        mpWLS->filter(disp16, grayL, filtered, dispR16);                // B4
        disp16 = filtered;
    }
    else
#endif
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
    PrioBoost boost;
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
    PrioBoost boost;
    std::unique_lock<std::mutex> lock(mMutexCloud);
    *mpGlobalCloud += *cloudWorld;
    mvGlobalDepth.insert(mvGlobalDepth.end(), depths.begin(), depths.end());
}

void PointCloudMapping::Impl::VoxelFilterGlobal()
{
    PrioBoost boost;
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
        // D1: maxrange <= 0 -> unlimited; discretize collapses rays per voxel.
        mpOctree->insertPointCloud(opc, octomap::point3d(t.x(), t.y(), t.z()),
                                   mCfg.octomapMaxRange > 0.0f ? mCfg.octomapMaxRange : -1.0,
                                   false, mCfg.octomapDiscretize);
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

static double msBetween(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b)
{
    return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(b - a).count();
}

/** Depth acquisition, voxel filter (sec. 3.2) and optional outlier removal (B3),
 *  all in the camera frame. */
void PointCloudMapping::Impl::PrepareCloud(KeyFrame *pKF, const cv::Mat &imA, const cv::Mat &imB,
                                           PointCloudT::Ptr &out, std::vector<float> &depths,
                                           double &msDepth, double &msVoxel)
{
    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();

    PointCloudT::Ptr cloudCam =
        (mSensor == System::RGBD || mSensor == System::IMU_RGBD)
            ? GenerateCloudRGBD(pKF, imA, imB)
            : GenerateCloudStereo(pKF, imA, imB);

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

    if (mCfg.outlierRemoval && cloudCamF->size() > static_cast<size_t>(mCfg.outlierMeanK))
    {
        PointCloudT::Ptr clean(new PointCloudT());
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloudCamF);
        sor.setMeanK(mCfg.outlierMeanK);
        sor.setStddevMulThresh(mCfg.outlierStddev);
        sor.filter(*clean);
        cloudCamF = clean;
    }

    depths.resize(cloudCamF->size());
    for (size_t i = 0; i < cloudCamF->size(); ++i) depths[i] = cloudCamF->points[i].z;
    out = cloudCamF;

    const auto t2 = clock::now();
    msDepth = msBetween(t0, t1);
    msVoxel = msBetween(t1, t2);
}

void PointCloudMapping::Impl::Run()
{
    using clock = std::chrono::steady_clock;
    int sinceFilter = 0;

    if (mCfg.lowPriority) ApplyLowPriority();

    while (true)
    {
        QueueItem item;
        {
            // Normal priority while mMutexQueue is held: tracking's Enqueue waits on it.
            PrioBoost boost;
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

        if (mbOffline)
        {
            // Offline + spill: the disk write happens here, off the tracking thread.
            if (item.pKF != nullptr)
            {
                SpillItem(item);
                PrioBoost boost;
                std::unique_lock<std::mutex> lock(mMutexQueue);
                mvOffline.push_back(std::move(item));
            }
            continue;
        }

        // In final-pose modes a keyframe culled meanwhile is kept: finalTwc()
        // follows its spanning-tree parent, exactly like FinalizeAll() does, so
        // online and offline integrate the same keyframe set. Thesis mode skips it.
        if (item.pKF == nullptr || (!mbStoreCam && item.pKF->isBad())) continue;

        const auto t0 = clock::now();

        PointCloudT::Ptr cloudCamF;
        std::vector<float> depths;
        double msDepth = 0.0, msVoxel = 0.0;
        PrepareCloud(item.pKF, item.imA, item.imB, cloudCamF, depths, msDepth, msVoxel);

        const auto t2 = clock::now();

        // B1/A5: keep the camera-frame cloud so Save() can use the final poses.
        if (mbStoreCam && cloudCamF->empty()) ++mnEmptyCloud;
        if (mbStoreCam && !cloudCamF->empty())
        {
            StoredCloud sc;
            sc.pKF = item.pKF;
            sc.cloud = cloudCamF;
            sc.depths = depths;
            std::unique_lock<std::mutex> ls(mMutexStored);
            mvStored.push_back(std::move(sc));
            mbFinalDone = false;
        }

        // Algorithm 1 line 21 / Algorithm 2 line 25: camera frame -> world. The
        // pose is read now, so it carries whatever the backend has optimised so
        // far for this keyframe (thesis behaviour). With reprojectOptimized the
        // world cloud is instead rebuilt at Save() and this is skipped.
        const bool needWorld = !mbReproject || (mCfg.octomapEnabled && !mbOctoAtEnd);
        Sophus::SE3f Twc;
        PointCloudT::Ptr cloudWorld(new PointCloudT());
        if (needWorld)
        {
            Twc = item.pKF->GetPoseInverse();
            if (!cloudCamF->empty())
                pcl::transformPointCloud(*cloudCamF, *cloudWorld, Twc.matrix());
        }

        if (!mbReproject && !cloudWorld->empty())
        {
            if (mCfg.probabilisticMerge) MergeProbabilistic(cloudWorld, depths);
            else                         MergeAppend(cloudWorld, depths);
        }

        const auto t3 = clock::now();

        if (mCfg.octomapEnabled && !mbOctoAtEnd && !cloudWorld->empty())
            InsertIntoOctomap(cloudWorld, Twc);

        const auto t4 = clock::now();

        // The probabilistic merge already enforces the minimum spacing (sec. 3.5
        // ends by noting redundant points are removed), so the global filter is
        // only needed on the plain-append path.
        if (!mbReproject && !mCfg.probabilisticMerge && ++sinceFilter >= 10)
        {
            VoxelFilterGlobal();
            sinceFilter = 0;
        }

        {
            std::unique_lock<std::mutex> lt(mMutexTiming);
            mvtDepth.push_back(msDepth);
            mvtVoxel.push_back(msVoxel);
            mvtMerge.push_back(msBetween(t2, t3));
            mvtOctomap.push_back(msBetween(t3, t4));
            mvtTotal.push_back(msBetween(t0, t4));
        }
    }

    {
        std::unique_lock<std::mutex> lock(mMutexFinish);
        mbFinished = true;
    }
}

// --- final-pose reconstruction (A3 / B1 / A5) -------------------------------

void PointCloudMapping::Impl::IntegrateFinal(const StoredCloud &sc, int &sinceFilter)
{
    if (!sc.pKF || !sc.cloud || sc.cloud->empty()) return;
    Sophus::SE3f Twc;
    bool rescued = false;
    if (!finalTwc(sc.pKF, Twc, &rescued)) { ++mnSkippedNoPose; return; }
    if (rescued) ++mnRescued;

    PointCloudT::Ptr cloudWorld(new PointCloudT());
    pcl::transformPointCloud(*sc.cloud, *cloudWorld, Twc.matrix());

    if (mbReproject)
    {
        if (mCfg.probabilisticMerge) MergeProbabilistic(cloudWorld, sc.depths);
        else
        {
            MergeAppend(cloudWorld, sc.depths);
            if (++sinceFilter >= 10) { VoxelFilterGlobal(); sinceFilter = 0; }
        }
    }
    if (mbOctoAtEnd) InsertIntoOctomap(cloudWorld, Twc);
}

void PointCloudMapping::Impl::FinalizeAll()
{
    std::unique_lock<std::mutex> lockFin(mMutexFinalize);
    if (!mbStoreCam) return;

    // The dense thread must be done: it appends to mvStored / the world cloud and
    // (offline+spill) still moves items into mvOffline.
    if (mThread.joinable())
    {
        bool fin;
        {
            std::unique_lock<std::mutex> lf(mMutexFinish);
            fin = mbFinished;
            if (!fin) mbFinishRequested = true;
        }
        if (!fin)
        {
            std::cerr << "[Dense] final-pose rebuild requested while the dense thread is "
                         "still running; draining it first." << std::endl;
            mQueueUpdated.notify_all();
            for (;;)
            {
                { std::unique_lock<std::mutex> lf(mMutexFinish); if (mbFinished) break; }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }

    // Idempotent: nothing new since the last rebuild.
    if (mbFinalDone.load())
    {
        std::unique_lock<std::mutex> lock(mMutexQueue);
        if (mvOffline.empty()) return;
    }

    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();

    // Offline: only now build the camera-frame clouds from the stored images.
    if (mbOffline)
    {
        std::vector<QueueItem> items;
        {
            std::unique_lock<std::mutex> lock(mMutexQueue);
            items.swap(mvOffline);
        }
        std::cout << "[Dense] offline: building clouds for " << items.size()
                  << " keyframes ..." << std::endl;
        for (auto &item : items)
        {
            if (item.pKF == nullptr || !UnspillItem(item)) continue;
            PointCloudT::Ptr cam;
            std::vector<float> depths;
            double msDepth = 0.0, msVoxel = 0.0;
            PrepareCloud(item.pKF, item.imA, item.imB, cam, depths, msDepth, msVoxel);
            {
                std::unique_lock<std::mutex> lt(mMutexTiming);
                mvtDepth.push_back(msDepth);
                mvtVoxel.push_back(msVoxel);
                mvtMerge.push_back(0.0);
                mvtOctomap.push_back(0.0);
                mvtTotal.push_back(msDepth + msVoxel);
            }
            item.imA.release();
            item.imB.release();
            if (cam->empty()) { ++mnEmptyCloud; continue; }
            StoredCloud sc;
            sc.pKF = item.pKF;
            sc.cloud = cam;
            sc.depths = std::move(depths);
            std::unique_lock<std::mutex> ls(mMutexStored);
            mvStored.push_back(std::move(sc));
        }
    }

    std::vector<StoredCloud> stored;
    {
        std::unique_lock<std::mutex> ls(mMutexStored);
        stored = mvStored;   // shared_ptr copies; clouds themselves are not duplicated
    }

    if (mbReproject)
    {
        std::unique_lock<std::mutex> lock(mMutexCloud);
        mpGlobalCloud.reset(mpPriorCloud ? new PointCloudT(*mpPriorCloud) : new PointCloudT());
        mvGlobalDepth = mvPriorDepth;
    }
    if (mbOctoAtEnd) ResetOctree();

    int sinceFilter = 0;
    mnRescued = 0;
    mnSkippedNoPose = 0;
    for (const auto &sc : stored) IntegrateFinal(sc, sinceFilter);
    if (mbReproject && !mCfg.probabilisticMerge) VoxelFilterGlobal();
    mbFinalDone = true;

    // Keyframes are never dropped silently, and clouds from different atlas
    // maps would share one frame here, so say so.
    {
        std::set<unsigned long> maps;
        for (const auto &sc : stored)
            if (sc.pKF && sc.pKF->GetMap()) maps.insert(sc.pKF->GetMap()->GetId());
        if (maps.size() > 1)
            std::cerr << "[Dense] WARNING: keyframes come from " << maps.size()
                      << " atlas maps; their clouds are merged in ONE frame, which is only "
                         "valid if the maps were merged." << std::endl;
        std::cout << "[Dense] keyframes: " << stored.size() << " stored, "
                  << mnRescued.load() << " culled (rescued via parent), "
                  << mnSkippedNoPose.load() << " lost (no live ancestor), "
                  << mnEmptyCloud.load() << " gave an empty cloud" << std::endl;
    }

    mFinalizeSeconds = std::chrono::duration<double>(clock::now() - t0).count();
    std::cout << "[Dense] final-pose reconstruction of " << stored.size()
              << " keyframes took " << std::fixed << std::setprecision(2)
              << mFinalizeSeconds << " s" << std::endl;
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
    mpImpl->Enqueue(std::move(item));
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
    mpImpl->Enqueue(std::move(item));
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

bool PointCloudMapping::NeedsFinalPoses() const
{
    return mpImpl && mpImpl->mCfg.enabled && mpImpl->mbStoreCam;
}

void PointCloudMapping::FinalizeOffline()
{
    if (!mpImpl || !mpImpl->mCfg.enabled) return;
    mpImpl->FinalizeAll();
}

void PointCloudMapping::Save()
{
    if (!mpImpl || !mpImpl->mCfg.enabled) return;

    // Must run before taking mMutexCloud: the merge functions lock it themselves.
    mpImpl->FinalizeAll();

    std::unique_lock<std::mutex> lock(mpImpl->mMutexCloud);
    const std::string base = mpImpl->mCfg.saveDirectory + "/" + mpImpl->mCfg.savePrefix;

    std::streamoff pcdBytes = -1, otBytes = -1, btBytes = -1;

    if (mpImpl->mpGlobalCloud && !mpImpl->mpGlobalCloud->empty())
    {
        const std::string pcd = base + "_cloud.pcd";
        if (pcl::io::savePCDFileBinary(pcd, *mpImpl->mpGlobalCloud) == 0)
        {
            std::cout << "[Dense] saved " << mpImpl->mpGlobalCloud->size()
                      << " points to " << pcd << std::endl;
            pcdBytes = fileSize(pcd);
        }
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
        {
            std::cout << "[Dense] saved octomap (" << mpImpl->mpOctree->size()
                      << " nodes) to " << ot << std::endl;
            otBytes = fileSize(ot);
        }
        else
            std::cerr << "[Dense] failed to write " << ot << std::endl;

        // D2: occupancy-only binary tree (no colour); much smaller than the .ot.
        if (mpImpl->mCfg.octomapWriteBt || mpImpl->mCfg.compressionReport)
        {
            const std::string bt = base + "_octomap.bt";
            if (mpImpl->mpOctree->writeBinary(bt))
            {
                std::cout << "[Dense] saved occupancy-only octomap to " << bt << std::endl;
                btBytes = fileSize(bt);
            }
            else
                std::cerr << "[Dense] failed to write " << bt << std::endl;
        }
    }

    // D2: compression ratios with both sides in a comparable format. The thesis
    // (Table IX) divides an ASCII .pcd by a binary .ot, which mixes formats.
    if (mpImpl->mCfg.compressionReport && pcdBytes > 0 && otBytes > 0)
    {
        const PointCloudT &cloud = *mpImpl->mpGlobalCloud;
        const std::string tmpAscii = base + "_cloud_ascii.tmp.pcd";
        const std::string tmpXyz   = base + "_cloud_xyz.tmp.pcd";
        pcl::PointCloud<pcl::PointXYZ> xyz;
        xyz.reserve(cloud.size());
        for (const auto &p : cloud.points) xyz.push_back(pcl::PointXYZ(p.x, p.y, p.z));
        pcl::io::savePCDFileASCII(tmpAscii, cloud);
        pcl::io::savePCDFileBinary(tmpXyz, xyz);
        const std::streamoff asciiBytes = fileSize(tmpAscii), xyzBytes = fileSize(tmpXyz);
        std::remove(tmpAscii.c_str());
        std::remove(tmpXyz.c_str());

        auto mb = [](std::streamoff b) { return static_cast<double>(b) / (1024.0 * 1024.0); };
        std::cout << std::fixed << std::setprecision(2)
                  << "[Dense] compression report (sizes in MiB, ratio = cloud / octomap)" << std::endl
                  << "  binary XYZRGB .pcd " << mb(pcdBytes) << "  vs colour .ot   " << mb(otBytes)
                  << "  -> " << static_cast<double>(pcdBytes) / otBytes << "x   [same format: binary, colour]" << std::endl;
        if (btBytes > 0 && xyzBytes > 0)
            std::cout << "  binary XYZ    .pcd " << mb(xyzBytes) << "  vs occupancy .bt " << mb(btBytes)
                      << "  -> " << static_cast<double>(xyzBytes) / btBytes << "x   [same format: binary, geometry only]" << std::endl;
        if (asciiBytes > 0)
            std::cout << "  ASCII XYZRGB  .pcd " << mb(asciiBytes) << "  vs colour .ot   " << mb(otBytes)
                      << "  -> " << static_cast<double>(asciiBytes) / otBytes
                      << "x   [MIXED formats; what a Table IX style ratio measures]" << std::endl;
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
    mpImpl->mpPriorCloud.reset(new PointCloudT(*loaded));
    mpImpl->mvPriorDepth = mpImpl->mvGlobalDepth;
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
    if (mpImpl->mCfg.queueLimit > 0)
        std::cout << "  Keyframes dropped (queueLimit=" << mpImpl->mCfg.queueLimit << "): "
                  << mpImpl->mnDropped << std::endl;
    if (mpImpl->mFinalizeSeconds > 0.0)
        std::cout << "  Final-pose rebuild at Save (s): " << mpImpl->mFinalizeSeconds << std::endl;
}

} // namespace ORB_SLAM3

#endif // WITH_DENSE_RECONSTRUCTION
