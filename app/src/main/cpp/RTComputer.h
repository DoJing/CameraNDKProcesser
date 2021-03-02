//
// Created by dojing on 2021/2/2.
//

#ifndef QRCODEDETECTOR_RTCOMPUTER_H
#define QRCODEDETECTOR_RTCOMPUTER_H
#include "PointDetector.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "ChessBoardDetector.h"
class RTComputer{
public:
    RTComputer();
    ~RTComputer();

    void clcPointsAndMatches(const cv::Mat& img);
    bool locateBoard(cv::Mat& img);
    bool filterMatches();
    bool clcRT(cv::Mat& R,cv::Mat& t);
    void initTemplatePoints();

private:
    std::vector<cv::Point2f> _board_points,_board_anchor;
    cv::Mat _boardpt_mat;
    std::vector<cv::Point2f> _img_points,_img_anchor;
    std::vector<cv::DMatch> _matches;
    CornerDetector _corner_detector;
    int _board_width,_board_height;
};
#endif //QRCODEDETECTOR_RTCOMPUTER_H
