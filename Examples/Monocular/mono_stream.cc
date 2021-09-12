/*
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <unistd.h>

#include <Converter.h>
#include <System.h>
using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv) {
  if (argc != 4) {
    cerr << endl
         << "Usage: ./mono_tum path_to_vocabulary path_to_settings "
            "stream url"
         << endl;
    return 1;
  }

  // Retrieve paths to images
  vector<double> vTimestamps;
  string strFile = "rtsp://" + string(argv[3]);
  // LoadImages(strFile, vstrImageFilenames, vTimestamps);
  cv::VideoCapture cap(strFile);
  int initialFrameCount = 100000;

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(initialFrameCount);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << initialFrameCount << endl << endl;
  // SLA:M.ActivateLocalizationMode();
  // Main loop
  cv::Mat im;
  for (;;) {
    // Read image from file
    cap.read(im);

    std::chrono::steady_clock::time_point ts = std::chrono::steady_clock::now();
    auto microsecondsUTC =
        std::chrono::duration_cast<std::chrono::microseconds>(
            ts.time_since_epoch())
            .count();

    double tframe = microsecondsUTC / 1000000.0;
    if (im.empty()) {
      cerr << endl
           << "Failed to load image at: " << string(argv[3]) << "/" << endl;
      return 1;
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Pass the image to the SLAM system
    SLAM.TrackMonocular(im, tframe);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
            .count();

    vTimesTrack.push_back(ttrack);
  }

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (int ni = 0; ni < vTimesTrack.size(); ni++) {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[vTimesTrack.size() / 2]
       << endl;
  cout << "mean tracking time: " << totaltime / vTimesTrack.size() << endl;

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  // SLAM.SaveMap("Slam_latest_Map.bin");

  return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps) {
  ifstream f;
  f.open(strFile.c_str());

  // skip first three lines
  string s0;
  getline(f, s0);
  getline(f, s0);
  getline(f, s0);

  while (!f.eof()) {
    string s;
    getline(f, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      string sRGB;
      ss >> t;
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenames.push_back(sRGB);
    }
  }
}
