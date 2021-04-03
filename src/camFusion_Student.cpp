
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    std::vector<cv::DMatch> matches_within_box;
    for (const auto& match : kptMatches)
        if (boundingBox.roi.contains(kptsCurr[match.trainIdx].pt))
            matches_within_box.push_back(match);

    std::vector<double> match_dists;
    match_dists.reserve(matches_within_box.size());
    for (const auto & match : matches_within_box) 
        match_dists.push_back(cv::norm(kptsPrev[match.queryIdx].pt - kptsCurr[match.trainIdx].pt));

    double sum_of_dists = 0;
    for (auto d : match_dists) sum_of_dists += d;
    double mean_dist = sum_of_dists / match_dists.size();

    for (int i = 0; i < matches_within_box.size(); ++i) {
        if (abs(match_dists[i] - mean_dist) < 20) { // Skip matches with distance too much greater than the mean
            boundingBox.keypoints.push_back(kptsCurr[matches_within_box[i].trainIdx]);
            boundingBox.kptMatches.push_back(matches_within_box[i]);
        }
    }

    // output to help with some debugging
    // std::cout << "clusterKptMatchesWithROI complete.\n";
    // std::cout << "Number of matches in frame: " << kptMatches.size() << std::endl;
    // std::cout << "Number of matches that are within bounding box: " << matches_within_box.size() <<std::endl;
    // std::cout << "Number of matches in box that are also not removed as outliers: " << boundingBox.kptMatches.size() << std::endl;
    // std::cout << "Match distances:\n  ";
    // for (auto d : match_dists) std::cout << d << ", ";
    // std::cout << std::endl;
    // std::cout << "Mean dist: " << mean_dist <<std::endl;

}

// Sorts list in place and then returns median
double median(std::vector<double>& a) {
    std::sort(a.begin(), a.end());
    if (a.size()%2) // if odd length
        return a[(a.size()-1)/2];
    else 
        return (a[int(a.size()/2 - 1)] + a[int(a.size()/2)]) / 2.0;
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImgPrev, cv::Mat *visImgCurr)
{
    // vector such that i^th component is ptr to kpt in kptsCurr that matches kptsPrev[i], or nullptr if there isn't one.
    std::vector<cv::KeyPoint*> match_map;
    for (int i = 0; i < kptsPrev.size(); ++i) {
        bool match_found = false;
        for (const auto& match : kptMatches) {
            if (match.queryIdx == i) {
                match_map.push_back(&kptsCurr[match.trainIdx]);
                match_found = true;
                break;
            }
        }
        if (!match_found)
            match_map.push_back(nullptr);
    }
    assert(match_map.size() == kptsPrev.size());
    // (I just realized that I could have double looped through the list of matches instead of making match_map, but whatever this is fine)

    std::vector<double> TTCs; // vector of TTC estimates we are going to collect for individual pairs of matches
    TTCs.reserve(kptsPrev.size() * kptsPrev.size());
    for (int i = 0; i < kptsPrev.size(); ++i) {
        if (match_map[i]==nullptr)
            continue;
        for (int j = 0; j < kptsPrev.size(); ++j) {
            if (match_map[j]==nullptr || i==j)
                continue;
            const auto& p0 = kptsPrev[i].pt;
            const auto& p1 = match_map[i]->pt;
            const auto& q0 = kptsPrev[j].pt;
            const auto& q1 = match_map[j]->pt;
            const double h0 = cv::norm(p0-q0);
            const double h1 = cv::norm(p1-q1);
            if (h0==h1) // if we are about to divide by 0
                TTCs.push_back(std::numeric_limits<double>::max()); // it's okay to use max b/c we will take median
            else
                TTCs.push_back(1/(frameRate * ((h1/h0) - 1)));
        }
    }
    
    // For robustness to garbage points, take the median ttc
    TTC = median(TTCs);
    
    // std::cout << "TTCs: ";
    // for (auto ttc : TTCs) std::cout << ttc << ", ";
    // std::cout << std::endl;
    

    // visualize results
    cv::Mat matchImg = visImgCurr->clone();
    cv::drawMatches(*visImgPrev, kptsPrev, *visImgCurr, kptsCurr, kptMatches,
                    matchImg, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    string windowName = "Matching keypoints between two camera images (best 50)";
    cv::namedWindow(windowName, 7);
    cv::imshow(windowName, matchImg);
}


// Given a list of lidar points, return the x coordinate of the lidar point with the least x coordinate
// This will try to be robust to outliers by considering only points whose x coordinate is part of a chain of length n+1 of
// points whose x coordinates are succesively within d of one another
LidarPoint* least_x_robust(std::vector<LidarPoint>& points, float d, int n) {
    
    // Sort points ascending by x coordinate
    std::sort(points.begin(), points.end(), [](const LidarPoint& a, const LidarPoint& b) {return a.x < b.x;});
    
    for (int i = 0; i < points.size() - n ; ++i) {
        
        // If points[i] up through points[i+n] are successively within d of each other, then we return points[i]
        
        bool all_within_d = true;
        for (int j = 1; j<=n; ++j)
            if (abs(points[i+j-1].x - points[i+j].x) >= d)
                all_within_d = false;
        
        if (all_within_d)
            return &(points[i]);

    }

    // Default to just giving the point with the least x coordinate
    return &(points[0]);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    const float d = 0.005; const int n = 3;
    LidarPoint* least_x_prev = least_x_robust(lidarPointsPrev, d, n);
    LidarPoint* least_x_curr = least_x_robust(lidarPointsCurr, d, n);

    // Output for debugging
    // std::cout << "Point xs:\n  ";
    // for (int i=0; i<lidarPointsCurr.size() && i<5;++i) {
    //     std::cout << lidarPointsCurr[i].x;
    //     if (i>0) std::cout << " (" << lidarPointsCurr[i].x - lidarPointsCurr[i-1].x << ")";
    //     std::cout << ", "; 
    // }
    // std::cout << "\nLeast x chosen: " << least_x_curr->x << "\n";

    double dx = least_x_prev->x - least_x_curr->x; // The amount by which x has decreased. A decreasing x makes this positive.
    TTC = least_x_curr->x / (dx * frameRate);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    for (const auto & prev_bbox : prevFrame.boundingBoxes) {

        const BoundingBox * curr_bbox_with_most_matches = nullptr; // The bounding box in the current frame that contains the most keypoints matched with prev_bbox
        int most_matches = 0; // The number of keypoint matches associated to the above
   
        for (const auto & curr_bbox : currFrame.boundingBoxes) {
            int num_matches = 0;
            for (const auto & match : matches) {
                if ( 
                    prev_bbox.roi.contains(prevFrame.keypoints[match.queryIdx].pt) &&
                    curr_bbox.roi.contains(currFrame.keypoints[match.trainIdx].pt)
                )
                    ++num_matches;
            }
            if (num_matches > most_matches) {
                most_matches = num_matches;
                curr_bbox_with_most_matches = &curr_bbox;
            }
        }

        if (most_matches > 0) {
            bbBestMatches.insert({prev_bbox.boxID, curr_bbox_with_most_matches->boxID});    
        }

    }

    // std::cout << "FINISHED MATCH BBOXES. FOUND " << bbBestMatches.size() << " MATCHES OF BBOXES.\n";
    // for (auto bbm : bbBestMatches)
    //     std::cout << "\t" << bbm.first << " - " << bbm.second << std::endl;

    
}
