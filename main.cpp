//
//  main.cpp
//  project1
//
//  Created by sxjscience on 15/3/7.
//  Copyright (c) 2015年 sxjscience. All rights reserved.
//

#include <iostream>
#include "cvtool.h"
#include <opencv2/opencv.hpp>

int main(int argc, const char * argv[]) {
    CVTool cvtool;
    cv::Mat img1 = cv::imread("data/a_complete.png");
    cv::Mat img2 = cv::imread("data/b.png");
    cvtool.detectFeatureSIFT(img1,1);
    cvtool.detectFeatureSIFT(img2,2);
    cvtool.matchFeatures();
    cvtool.computeFundMatrix();
//    cvtool.detectFeatureSURF(img,1,10);
//    cvtool.detectFeatureSURF(img,1,100);
//    cvtool.detectFeatureSURF(img,1,500);
//    cvtool.detectFeatureSURF(img,1,1000);
//    cvtool.detectFeatureMSER(img,1);
//    cvtool.detectFeatureHaris(img);
    std::cout << "Hello, World!\n";
    return 0;
}
