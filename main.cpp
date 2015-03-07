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
    // insert code here...
    CVTool cvtool;
    cv::Mat img = cv::imread("data/a_complete.png");
    cvtool.detectFeatureSIFT(img);
    std::cout << "Hello, World!\n";
    return 0;
}
