
#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    cv::Mat descSource2, descRef2; // Copies of descSource and descRef whose format might be adjusted if FLANN is used

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);
        descSource2 = descSource;
        descRef2 = descRef;
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        if (descSource.type() != CV_32F) {
            descSource.convertTo(descSource2, CV_32F);
            descRef.convertTo(descRef2, CV_32F);
        }
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource2, descRef2, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        vector<vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource2, descRef2, knn_matches, 2);

        for (auto knn_match : knn_matches)
            if (knn_match[0].distance < 0.8 * knn_match[1].distance)
                matches.push_back(knn_match[0]);
    }
}

// -> BRIEF, ORB, FREAK, AKAZE, SIFT
// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType, bool silent)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0) {
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0) {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0) {
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("FREAK") == 0) {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("AKAZE") == 0) {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0) {
        // extractor = cv::xfeatures2d::SiftDescriptorExtractor::create();
        cerr << "ERROR: SIFT has been disabled in this environment.\n";
    }
    else
        cerr << "ERROR: Invalid descriptorType." << endl;

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    if (!silent)
        cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis, bool silent)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    if (!silent)
        cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis, bool silent) {
        // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize ?? blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    double t0 = get_ticks_ms(); // time this

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat()); // shifts corner response values into 0-255 range
    cv::convertScaleAbs(dst_norm, dst_norm_scaled); // converts to 8 bit from the original CV_32FC1 (there is no scaling or abs; the name of the fn is confusing)

    for (int i=0; i<dst_norm.rows; ++i)
    {
        for (int j=0; j<dst_norm.cols; ++j) {
            auto & r = dst_norm.at<float>(i,j);
            if (r>minResponse)
                keypoints.emplace_back(float(j),float(i),2*apertureSize,-1,r);

        }
    }

    float repel_dist = 6.0;

    double t1 = get_ticks_ms();
    if (!silent)
        cout << "Harris corner detection initially found " << keypoints.size() << " keypoints. It took " << t1-t0 << "ms.\n";

    for (auto it = keypoints.begin(); it!=keypoints.end();) {
        bool delete_this_one = false;
        for (const auto & p : keypoints){
            if (dst_norm.at<float>(it->pt.y,it->pt.x) && (cv::norm(p.pt - it->pt) < repel_dist)) {
                if (p.pt == it->pt)
                    continue;
                delete_this_one=true;
                break;
            }
        }
        if (delete_this_one) {
            it = keypoints.erase(it);
        }
        else {
            ++it;
        }
    }

    double t2 = get_ticks_ms();
    if (!silent)
        cout << "After nonmax suppression, " << keypoints.size() << " remain. Nonmax suppression took " << t2-t1 << "ms\n.";

    if (bVis) {
        string windowName = "Harris Corner Detection Results";
        cv::namedWindow(windowName, 1);
        cv::Mat visImage = dst_norm_scaled.clone();
        cv::drawKeypoints(dst_norm_scaled, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow(windowName, visImage);
        cv::waitKey(0);
    }

}


void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis, bool silent) {

    double t0 = get_ticks_ms();

    cv::Ptr<cv::Feature2D> detector;
    if (detectorType.compare("FAST") == 0)
        detector = cv::FastFeatureDetector::create(28);
    else if (detectorType.compare("BRISK") == 0)
        detector = cv::BRISK::create();
    else if (detectorType.compare("ORB") == 0)
        detector = cv::ORB::create(1000);
    else if (detectorType.compare("AKAZE") == 0)
        detector = cv::AKAZE::create();
    else if (detectorType.compare("SIFT") == 0)
        // detector = cv::xfeatures2d::SIFT::create();
        cerr << "ERROR: SIFT has been disabled in this environment.\n";
    else {
        cerr << "ERROR: Invalid detectorType requested.\n";
    }
    
    detector->detect(img, keypoints);
    if (!silent)
        cout << detectorType << " keypoint finder with " << keypoints.size() << " keypoints in " << get_ticks_ms() - t0 << " ms" << endl;

    if (bVis) {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = detectorType + " keypoint finder  Results";
        cv::namedWindow(windowName, 1);
        cv::imshow(windowName, visImage);
        cv::waitKey(0);
    }
}