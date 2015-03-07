#include "cvtool.h"
#include "assert.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

CVTool::CVTool() {
    
}

void CVTool::setImage(int imgID, const cv::Mat & img, const VKP &keyPoints, const cv::Mat &descriptor) {
    if(imgID ==1){
        keyPoints1 = keyPoints;
        descriptor1 = descriptor;
        image1_ = img;
    }
    else if(imgID == 2){
        keyPoints2 = keyPoints;
        descriptor2 = descriptor;
        image2_ = img;
    }
    else {
        assert(0);
    }

}

void CVTool::detectFeatureSIFT(const cv::Mat &img,int imgID,int quiet, int nfeature, int nOctaveLayers, double contrastThreshold, double edgeThreshold, double sigma) {
    cv::SIFT sift(nfeature,nOctaveLayers,contrastThreshold,edgeThreshold,sigma);
    cv::Mat dummy;
    VKP keyPoints;
    cv::Mat descriptor;
    sift(img,dummy,keyPoints,descriptor);
    setImage(imgID,img,keyPoints,descriptor);
    if(!quiet) {
        cv::Mat output;
        cv::drawKeypoints(img, keyPoints, output);
        cv::namedWindow("SIFT");
        cv::imshow("SIFT", output);
        cv::waitKey();
    }
    return ;
}

void CVTool::detectFeatureSURF(const cv::Mat &img, int imgID, int quiet,double hessianThreshold, int nOctave, int nOctaveLayers, bool extended, bool upright) {
    cv::SURF surf(hessianThreshold,nOctaveLayers,extended,upright);
    cv::Mat dummy;
    VKP keyPoints;
    cv::Mat descriptor;
    surf(img,dummy,keyPoints,descriptor);
    setImage(imgID,img,keyPoints,descriptor);
    if(!quiet) {
        std::cout<<descriptor.rows<<std::endl;
        std::cout<<descriptor.cols<<std::endl;
        cv::Mat output;
        cv::drawKeypoints(img, keyPoints, output);
        cv::namedWindow("SURF");
        cv::imshow("SURF", output);
        cv::waitKey();
    }
    return ;
}


void CVTool::detectFeatureMSER(const cv::Mat &img, int imgID,int quiet,int _delta, int _min_area, int _max_area, double _max_variation, double _min_diversity, int _max_evolution, double _area_threshold, double _min_margin, int _edge_blur_size) {
    cv::MSER mser(_delta,_min_area,_max_area,_max_variation,_min_diversity,_max_variation,_area_threshold,_min_margin,_edge_blur_size);
    cv::Mat dummy;
    std::vector<std::vector<cv::Point> > keyContour;
    VKP keyPoints;
    mser(img,keyContour,dummy);
    std::cout<<keyContour.size()<<std::endl;
    for (int i=0; i<keyContour.size(); i++) {
        for (int j=0; j<keyContour.size(); j++) {
            cv::KeyPoint kp;
            kp.pt = keyContour[i][j];
            kp.angle = -1;
            std::cout<<keyContour[i][j].x<<" "<<keyContour[i][j].y<<std::endl;
            keyPoints.push_back(kp);
        }
        
    }
    if(!quiet) {
        cv::Mat output;
        cv::drawKeypoints(img, keyPoints, output);
        cv::namedWindow("MSER");
        cv::imshow("MSER", output);
        cv::waitKey();
    }    
}

void CVTool::detectFeatureHaris(const cv::Mat &img, int imgID,int quiet) {
    cv::Ptr<cv::FeatureDetector> detectT = cv::FeatureDetector::create("HARRIS");
    cv::Ptr<cv::DescriptorExtractor> desciptorT = cv::DescriptorExtractor::create("SIFT");
    VKP keyPoints;
    cv::Mat descriptor;
    detectT->detect(img, keyPoints);
    desciptorT->compute(img, keyPoints, descriptor);
    setImage(imgID,img,keyPoints,descriptor);
    if(!quiet) {
        std::cout<<descriptor.rows<<std::endl;
        std::cout<<descriptor.cols<<std::endl;
        cv::Mat output;
        cv::drawKeypoints(img, keyPoints, output);
        cv::namedWindow("Harris");
        cv::imshow("Harris", output);
        cv::waitKey();
    }

}

void CVTool::matchFeatures(int quiet) {
    assert(descriptor1.cols == descriptor2.cols);
    assert( (keyPoints1.size() == descriptor1.rows) && (keyPoints2.size() == descriptor2.rows) );
    matches.clear();
    goodMatches.clear();
    cv::FlannBasedMatcher matcher;
    matcher.match(descriptor1, descriptor2, matches);
    for (int i=0; i<matches.size(); i++) {
        std::cout<<matches[i].queryIdx<<" "<<matches[i].trainIdx<<" "<<matches[i].distance<<std::endl;
    }
    double minDist = 100000;
    double maxDist = -10000;
    for( int i = 0; i < descriptor1.rows; i++ ) {
        if(matches[i].distance < minDist) {
            minDist = matches[i].distance;
        }
        if(matches[i].distance > maxDist) {
            maxDist = matches[i].distance;
        }
    }
    std::cout<<minDist<<std::endl;
    std::cout<<maxDist<<std::endl;

    for( int i = 0; i < descriptor1.rows; i++ ) {
        if(std::max(2*minDist, 0.02) >= matches[i].distance ) {
            goodMatches.push_back(matches[i]);
        }
    }
    
    /*  Sorting */
    for (int i=0; i<goodMatches.size(); i++) {
        for (int j=0; j<goodMatches.size()-i-1; j++) {
            if (goodMatches[j].distance <goodMatches[j+1].distance) {
                cv::DMatch temp = goodMatches[j];
                goodMatches[j] = goodMatches[j+1];
                goodMatches[j+1] = temp;
            }
        }
    }
    if (!quiet) {
        cv::Mat output;
        cv::drawMatches( image1_, keyPoints1, image2_, keyPoints2,
                        goodMatches, output, cv::Scalar::all(-1), cv::Scalar::all(-1),
                        std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        cv::namedWindow("Matching");
        cv::imshow("Matching", output);
        cv::waitKey();
    }
    
}


void CVTool::computeFundMatrix() {
    matchFeatures(1);
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    for (int i=0; i<goodMatches.size(); i++) {
        points1.push_back(keyPoints1[goodMatches[i].queryIdx].pt);
        points2.push_back(keyPoints2[goodMatches[i].queryIdx].pt);
    }
    fundMat = cv::findFundamentalMat(points1,points2);
    std::vector<cv::Point2f> interestPts1;
    std::vector<cv::Point2f> interestPts2;
    std::vector<cv::Vec<float,3> >  lines1;
    std::vector<cv::Vec<float,3> >  lines2;
    for (int i=0; i<8; i++) {
        interestPts1.push_back(keyPoints1[goodMatches[i].queryIdx].pt);
        interestPts2.push_back(keyPoints2[goodMatches[i].trainIdx].pt);
    }
    cv::computeCorrespondEpilines(interestPts1,1,fundMat,lines1);
    cv::computeCorrespondEpilines(interestPts2,2,fundMat,lines2);
    for (int i=0; i<interestPts2.size(); i++) {
        interestPts2[i].x = interestPts2[i].x + image1_.cols;
    }
    cv::Mat outImg(image1_.rows,image1_.cols*2,CV_8UC3);
    cv::Rect rect1(0,0, image1_.cols, image1_.rows);
    cv::Rect rect2(image1_.cols, 0, image1_.cols, image1_.rows);
    cv::RNG rng;
    if (image1_.type() == CV_8U)
    {
        cv::cvtColor(image1_, outImg(rect1), CV_GRAY2BGR);
        cv::cvtColor(image2_, outImg(rect2), CV_GRAY2BGR);
    }
    else
    {
        image1_.copyTo(outImg(rect1));
        image2_.copyTo(outImg(rect2));
    }
    for (int i=0; i<interestPts1.size(); i++) {
        cv::Scalar color(rng(256),rng(256),rng(256));
        cv::line(outImg,
                 cv::Point(0,-lines1[i][2]/lines1[i][1]),
                 cv::Point(image1_.cols,-(lines1[i][2]+lines1[i][0]*image1_.cols)/lines1[i][1]),
                 color);
        cv::circle(outImg, interestPts1[i], 3, color, -1, CV_AA);
        
        cv::line(outImg,
                 cv::Point(image1_.cols,-lines2[i][2]/lines2[i][1]),
                 cv::Point(image1_.cols+image2_.cols,-(lines2[i][2]+lines2[i][0]*image2_.cols)/lines2[i][1]),
                 color);
        cv::circle(outImg, interestPts2[i], 3, color, -1, CV_AA);
    }
    cv::namedWindow("Epi");
    cv::imshow("Epi", outImg);
    cv::waitKey();
}
