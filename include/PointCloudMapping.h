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
