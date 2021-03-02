#ifndef QRCODEDETECTOR_POINTDETECTOR_H
#define QRCODEDETECTOR_POINTDETECTOR_H
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#define  mtype CV_32F
#define  dtype  float
class CornerDetector
{
public:
    CornerDetector();
    ~CornerDetector();
    void detectCorners(const cv::Mat& Src, std::vector<cv::Point2f>& outputCorners, bool isrefine = true);
    void detectCorners(const cv::Mat& Src, std::vector<cv::Point2f>& corners_vec, cv::Mat& corners_mat, bool isrefine = true);
private:
    dtype normpdf(dtype dist, dtype mu, dtype sigma);

    void getMin(cv::Mat src1, cv::Mat src2, cv::Mat &dst);

    void getMax(cv::Mat src1, cv::Mat src2, cv::Mat &dst);

    void createkernel(float angle1, float angle2, int kernelSize, cv::Mat &kernelA, cv::Mat &kernelB, cv::Mat &kernelC, cv::Mat &kernelD);

    void nonMaximumSuppression(cv::Mat& inputCorners, std::vector<cv::Point2f>& outputCorners, int patchSize, dtype threshold, int margin);
    std::vector<cv::Point2f> cornerPoints;
    cv::Mat kernelA1, kernelB1, kernelC1, kernelD1;
};
#endif