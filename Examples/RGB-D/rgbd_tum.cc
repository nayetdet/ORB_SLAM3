/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<cstdlib>
#include<cstdio>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    // 4 arguments: one sequence (the original usage). 4 + 2k arguments: a chain of k+1 sequences processed in ONE
    // process, so that ORB-SLAM3's Atlas merges the maps itself (same mechanism as Examples/Stereo/stereo_euroc).
    if(argc < 5 || (argc % 2) == 0)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association "
                        "(path_to_sequence_2 path_to_association_2 ... path_to_sequence_N path_to_association_N)" << endl;
        return 1;
    }
    const int num_seq = (argc-3)/2;

    // Opt-in: ORB_NO_PACING=1 feeds the frames as fast as the system accepts them instead of sleeping to the
    // dataset timestamps (unset or any other value: unchanged behaviour). Used to measure throughput (FPS).
    const char *envNoPacing = getenv("ORB_NO_PACING");
    const bool bNoPacing = (envNoPacing != NULL && string(envNoPacing) == "1");

    // Retrieve paths to images
    vector< vector<string> > vstrImageFilenamesRGB(num_seq);
    vector< vector<string> > vstrImageFilenamesD(num_seq);
    vector< vector<double> > vTimestamps(num_seq);
    vector<int> nImagesSeq(num_seq);
    int nImages = 0;
    for(int seq=0; seq<num_seq; seq++)
    {
        string strAssociationFilename = string(argv[(2*seq) + 4]);
        LoadImages(strAssociationFilename, vstrImageFilenamesRGB[seq], vstrImageFilenamesD[seq], vTimestamps[seq]);

        // Check consistency in the number of images and depthmaps
        nImagesSeq[seq] = vstrImageFilenamesRGB[seq].size();
        if(vstrImageFilenamesRGB[seq].empty())
        {
            cerr << endl << "No images found in provided path." << endl;
            return 1;
        }
        else if(vstrImageFilenamesD[seq].size()!=vstrImageFilenamesRGB[seq].size())
        {
            cerr << endl << "Different number of images for rgb and depth." << endl;
            return 1;
        }
        nImages += nImagesSeq[seq];
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    if(num_seq == 1)
        cout << "Images in the sequence: " << nImages << endl << endl;
    else
        cout << "Sequences in the chain: " << num_seq << ", images in total: " << nImages << endl << endl;
    if(bNoPacing)
        cout << "ORB_NO_PACING=1: not sleeping to the dataset timestamps" << endl;

    // Wall-clock (monotonic) time of the whole frame loop, image loading and pacing sleeps included.
    const std::chrono::steady_clock::time_point tLoop0 = std::chrono::steady_clock::now();

    // Main loop
    cv::Mat imRGB, imD;
    int nProcessed = 0;
    for(int seq=0; seq<num_seq; seq++)
    {
        const string strSeqPath = string(argv[(2*seq) + 3]);
        for(int ni=0; ni<nImagesSeq[seq]; ni++, nProcessed++)
        {
            // Read image and depthmap from file
            imRGB = cv::imread(strSeqPath+"/"+vstrImageFilenamesRGB[seq][ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
            imD = cv::imread(strSeqPath+"/"+vstrImageFilenamesD[seq][ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
            double tframe = vTimestamps[seq][ni];

            if(imRGB.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << strSeqPath << "/" << vstrImageFilenamesRGB[seq][ni] << endl;
                return 1;
            }

            if(imageScale != 1.f)
            {
                int width = imRGB.cols * imageScale;
                int height = imRGB.rows * imageScale;
                cv::resize(imRGB, imRGB, cv::Size(width, height));
                cv::resize(imD, imD, cv::Size(width, height));
            }

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

            // Pass the image to the SLAM system
            SLAM.TrackRGBD(imRGB,imD,tframe);

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[nProcessed]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImagesSeq[seq]-1)
                T = vTimestamps[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestamps[seq][ni-1];

            if(!bNoPacing && ttrack<T)
                usleep((T-ttrack)*1e6);
        }

        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }
    }

    // Always printed (paced or not): frames / wall-clock time of the frame loop, Shutdown() excluded.
    {
        const double loopSeconds = std::chrono::duration_cast<std::chrono::duration<double> >(std::chrono::steady_clock::now() - tLoop0).count();
        char line[160];
        snprintf(line, sizeof(line), "Processed %d frames in %.2f s = %.2f fps", nImages, loopSeconds,
                 loopSeconds > 0.0 ? nImages / loopSeconds : 0.0);
        cout << line << endl;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    if(num_seq > 1)
    {
        // Chain only: also the EuRoC-style frame trajectory, which holds the frames of the biggest map only and
        // prints "There are N maps in the atlas" (N > 1 = the sequences did not merge). Timestamps are in ns.
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory_euroc.txt");
    }

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
