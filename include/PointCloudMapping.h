/**
* This file is part of ORB-SLAM3
*
* Dense Reconstruction thread, reproducing the methodology of:
*   Hanxiang Zhang, "Dense Reconstruction from Visual SLAM with Probabilistic
*   Multi-Sequence Merging", MASc thesis, Dalhousie University, December 2023.
*
* Implements:
*   - Algorithm 1 (Fig. 13): RGB-D point cloud generation
*   - Algorithm 2 (Fig. 15): stereo point cloud generation
*   - Section 3.4: Octomap conversion from the dense point cloud
*   - Section 3.5, eq. (34)/(35): depth-based confidence and the probabilistic
*     merge used for multi-sequence operation
*
* This header deliberately pulls in neither PCL nor octomap. PCL 1.15 requires
* C++17 while the rest of ORB-SLAM3 builds as C++11, so every PCL type lives
* behind the opaque Impl below and only PointCloudMapping.cc is compiled as
* C++17. System.cc and Tracking.cc can therefore include this freely.
*
* When the build cannot find PCL or octomap the whole feature compiles to
* no-ops and the system behaves exactly like stock ORB-SLAM3.
*/

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include <string>
#include <opencv2/core/core.hpp>

namespace ORB_SLAM3
{

class KeyFrame;

class PointCloudMapping
{
public:
    /** Parameters, all read from the .yaml settings file (sec. 3.2-3.5).
     *
     *  The thesis does not publish values for most of these, so the defaults are
     *  documented choices, not reproductions. Keeping them in the settings file
     *  means a run is reproducible from that file alone.
     */
    struct Config
    {
        bool  enabled            = false;

        // --- dense point cloud (sec. 3.2 / 3.3) ---
        float resolution         = 0.01f;  // voxel grid leaf size [m]
        float depthMin           = 0.5f;   // d_min, eq. (34) [m]
        float depthMax           = 5.0f;   // d_max, eq. (34) [m]

        // --- probabilistic merge (sec. 3.5, eq. 34/35) ---
        bool  probabilisticMerge = false;
        float probMax            = 0.9f;   // P_max at d_min; P_min = 1 - P_max
        float mergeDistance      = 0.0f;   // 0 -> use `resolution`

        // --- stereo only (sec. 3.3) ---
        int   sgbmNumDisparities = 96;     // must be divisible by 16
        int   sgbmBlockSize      = 9;      // must be odd
        int   roiMarginX         = 0;      // ROI, Algorithm 2 line 13
        int   roiMarginY         = 0;
        bool  colorizeByDepth    = true;   // rainbow by depth; far = red (sec. 4.2)
        cv::Vec3b solidColor     = cv::Vec3b(200, 200, 200);  // BGR, when colorize off

        // --- octomap (sec. 3.4) ---
        bool  octomapEnabled     = true;
        float octomapResolution  = 0.05f;  // leaf size [m]
        // Ray casting from the camera centre carves free space, which is what
        // gives eq. (31)-(33)'s log-odds update something to decrease. It also
        // creates a free node for every voxel along every ray, which makes the
        // .ot file LARGER than the .pcd -- Table IX reports the octomap 4.8-6.5x
        // SMALLER, so the thesis cannot have been carving. Default off to match
        // its numbers; turn on if you want a navigable occupancy map.
        bool  octomapRayCast     = false;

        // --- opt-in extensions (see melhorias.md). Every default below keeps the
        //     thesis-faithful behaviour; nothing here changes a run unless set. ---

        // A1: max keyframes waiting for the dense thread. 0 = unlimited (thesis).
        // When exceeded the OLDEST queued keyframe is dropped, never the newest.
        // Ignored in offline mode (which stores everything).
        int   queueLimit         = 0;

        // A2: run the dense thread at SCHED_IDLE (fallback nice +10) so it only
        // gets CPU nobody else wants. Linux only; a no-op elsewhere.
        bool  lowPriority        = false;

        // A3: "online" (thesis) or "offline". Offline only stores the keyframe
        // pointer and its images while SLAM runs; every cloud is built by
        // FinalizeOffline() (called automatically by Save()) with the final,
        // globally optimised keyframe poses. Implies reprojectOptimized and
        // octomapAtEnd.
        bool  offline            = false;
        // Directory to spill the stored offline images to instead of RAM
        // (empty = keep in RAM). Files are deleted once consumed.
        std::string offlineSpillDir = "";

        // B1: keep per-keyframe clouds in the camera frame and transform them to
        // the world with the FINAL keyframe poses at Save() time, so the dense
        // map follows loop closure / global BA. The incremental world cloud is
        // then not built during the run (CloudSize() only reflects a prior cloud
        // until Save()). Implies octomapAtEnd.
        bool  reprojectOptimized = false;

        // A5: build the octomap once at Save() from the stored per-keyframe
        // clouds with final poses, instead of per keyframe on the dense thread.
        bool  octomapAtEnd       = false;

        // D1: only used when octomapRayCast is on. Max ray length [m], <=0 means
        // unlimited (octomap default); and octomap's `discretize`, which groups
        // rays ending in the same voxel.
        float octomapMaxRange    = -1.0f;
        bool  octomapDiscretize  = false;

        // B3: pcl::StatisticalOutlierRemoval on each keyframe cloud after the
        // voxel filter.
        bool  outlierRemoval     = false;
        int   outlierMeanK       = 20;
        float outlierStddev      = 1.0f;

        // B4: cv::ximgproc weighted-least-squares disparity filter (stereo only;
        // needs opencv_contrib ximgproc at build time, else it warns and is off).
        bool  wlsFilter          = false;
        double wlsLambda         = 8000.0;
        double wlsSigmaColor     = 1.5;

        // D2: also write <prefix>_octomap.bt (occupancy only, no colour) and print
        // size ratios with both sides in the same format.
        bool  octomapWriteBt     = false;
        bool  compressionReport  = false;

        // --- output ---
        std::string saveDirectory = ".";
        std::string savePrefix    = "dense";
        std::string loadCloud     = "";    // prior cloud for multi-sequence runs
    };

    /** Reads the Dense.* keys. Returns a disabled config if the file has none. */
    static Config LoadConfig(const std::string &settingsFile);

    /** True when the binary was built with PCL and octomap available. */
    static bool Available();

    PointCloudMapping(const Config &cfg, int sensor);
    ~PointCloudMapping();

    PointCloudMapping(const PointCloudMapping&) = delete;
    PointCloudMapping& operator=(const PointCloudMapping&) = delete;

    /** Algorithm 1 lines 1-4: queue a keyframe's colour and depth image. */
    void InsertKeyFrameRGBD(KeyFrame *pKF, const cv::Mat &imColor, const cv::Mat &imDepth);

    /** Algorithm 2 lines 1-5: queue a keyframe's rectified stereo pair. */
    void InsertKeyFrameStereo(KeyFrame *pKF, const cv::Mat &imLeft, const cv::Mat &imRight);

    void RequestFinish();
    bool IsFinished();

    /** Offline mode (A3) / reprojectOptimized (B1) / octomapAtEnd (A5): builds
     *  the world cloud and octomap from the final keyframe poses. Must be called
     *  after the last keyframe was inserted and after global BA finished, i.e.
     *  after RequestFinish()/IsFinished(). Idempotent. Save() calls it itself, so
     *  a System that already does RequestFinish -> IsFinished -> Save needs no
     *  change; call it explicitly only to time or order the step yourself. No-op
     *  in plain online mode. */
    void FinalizeOffline();

    /** True when Save() rebuilds the cloud/octomap from keyframe poses, i.e. the
     *  caller must let LocalMapping/LoopClosing/GBA finish before Save(). */
    bool NeedsFinalPoses() const;

    /** Writes <saveDirectory>/<savePrefix>_cloud.pcd and _octomap.ot. */
    void Save();

    /** Loads a previously saved cloud so a later sequence merges into it
     *  (sec. 3.5, multi-sequence operation). */
    bool LoadCloud(const std::string &pcdFile);

    size_t CloudSize();

    /** Mirrors the Dense Reconstruction block of Table X. */
    void PrintTimingSummary();

private:
    struct Impl;
    Impl *mpImpl;
};

} // namespace ORB_SLAM3

#endif // POINTCLOUDMAPPING_H
