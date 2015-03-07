#ifndef CVTOOL_H
#define CVTOOL_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>


#define VKP std::vector<cv::KeyPoint>
#define CVTOOLDEBUG 1
/**
 * @brief The CVTool class
 * This class contains all the functions you should implement in the project.
 * The member functions are initially defined, but you should define the parameters and return value yourself.
 * The name of the public member functions cannot be modified.
 * You can add any protected functions or members.
 */
class CVTool
{
public:
    CVTool();
    /**
     * @author sxjscience
     *
     *
     */
    void detectFeatureSIFT(const cv::Mat &img, int imgID = 1, int quiet = 0,int nfeature = 0, int nOctaveLayers = 3, double contrastThreshold = 0.04, double edgeThreshold=10, double sigma=1.6);

    void detectFeatureSURF(const cv::Mat &img, int imgID = 1, int quiet = 0, double hessianThreshold = 0.1, int nOctave =4, int nOctaveLayers=2, bool extended=true, bool upright=false);

    void detectFeatureMSER(const cv::Mat &img, int imgID = 1,int quiet = 0,int _delta=5, int _min_area=60, int _max_area=14400, double _max_variation=0.25, double _min_diversity=.2, int _max_evolution=200, double _area_threshold=1.01, double _min_margin=0.003, int _edge_blur_size=5 );

    void detectFeatureHaris(const cv::Mat &img, int imgID = 1,int quiet = 0);

    /**
     * @brief for function 2.
     */
    void matchFeatures(int quiet = 0);

    void visualizeMatching();


    /**
     * @brief repairImage
     * @param damaged_img_a
     * @param complete_img_b
     * @return repaired_img_a
     */
    cv::Mat repairImage(const cv::Mat & damaged_img_a, const cv::Mat & complete_img_b);


    /**
     * @brief visualizeDiffence
     * @param repaired_img_a
     * @param complete_img_a
     * compute the difference between the repaired image and the complete image
     */
    void visualizeDiffence(const cv::Mat & repaired_img_a, const cv::Mat & complete_img_a);

    /**
     * @brief for function 4.
     */
    void computeFundMatrix();

    void visualizeEpipolarLine();

protected:
    /**
      * You can add any protected function to help.
      */
    void setImage(int imgID, const cv::Mat &img, const VKP &keyPoints, const cv::Mat &descriptor);
protected:
    /**
     * @brief image1_, image2_ are the two input images for testing
     */
    cv::Mat image1_, image2_;
    VKP keyPoints1,keyPoints2;
    cv::Mat descriptor1, descriptor2;
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> goodMatches;
    cv::Mat fundMat;
    /**
      * You can add any other members to help.
      */
    
};

#endif // CVTOOL_H
