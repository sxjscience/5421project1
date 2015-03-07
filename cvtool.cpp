#include "cvtool.h"
#include "assert.h"
#include <iostream>

CVTool::CVTool() {
    
}

void CVTool::detectFeatureSIFT(const cv::Mat &img,int imgID, int nfeature, int nOctaveLayers, double contrastThreshold, double edgeThreshold, double sigma) {
    cv::SIFT detector(nfeature,nOctaveLayers,contrastThreshold,edgeThreshold,sigma);
    cv::Mat dummy;
    VKP keyPoints;
    cv::Mat descriptor;
    detector(img,dummy,keyPoints,descriptor);
    std::cout<<descriptor.rows<<std::endl;
    std::cout<<descriptor.cols<<std::endl;
    if(imgID ==1){
        keyPoints1 = keyPoints;
        descriptor1 = descriptor;
    }
    else if(imgID == 2){
        keyPoints2 = keyPoints;
        descriptor2 = descriptor;
    }
    else {
        assert(0);
    }
    if(CVTOOLDEBUG) {
        cv::Mat output;
        cv::drawKeypoints(img, keyPoints, output);
        cv::namedWindow("Win");
        cv::imshow("win", output);
        cv::waitKey();
    }
    return ;
}