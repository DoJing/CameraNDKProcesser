//
// Created by dojing on 2021/2/2.
//
#include "RTComputer.h"
using namespace cv;
using namespace std;
RTComputer::RTComputer() {}
RTComputer::~RTComputer() {}
void RTComputer::initTemplatePoints() {

    _board_height=540;
    _board_width=960;
    int points_num = 54;
    float points[108]{217.701,153.378,217.697,200.655,217.694,248.224,217.698,295.828,217.704,343.364,217.687,390.63,287.571,153.368,287.566,200.65,287.557,248.238,287.559,295.814,287.563,343.407,287.568,390.623,357.383,153.381,357.376,200.627,357.387,248.222,357.381,295.822,357.401,343.384,357.383,390.622,427.805,153.378,427.791,200.624,427.772,248.226,427.791,295.824,427.8,343.389,427.802,390.628,497.538,153.366,497.544,200.634,497.54,248.228,497.545,295.803,497.535,343.408,497.526,390.612,567.571,153.389,567.554,200.622,567.548,248.222,567.552,295.805,567.542,343.389,567.572,390.628,637.735,153.368,637.722,200.645,637.748,248.214,637.73,295.835,637.743,343.361,637.719,390.625,707.564,153.378,707.579,200.614,707.584,248.225,707.579,295.803,707.583,343.377,707.582,390.617};
    _board_points.resize(points_num);
    _boardpt_mat = Mat(points_num,2,CV_32FC1);
    for(int i=0;i<54;i++){
        _board_points[i].x=points[i*2];
        _board_points[i].y=points[i*2+1];
        _boardpt_mat.at<float>(i,0)=_board_points[i].x;
        _boardpt_mat.at<float>(i,1)=_board_points[i].y;
    }
    _board_anchor.emplace_back(54.5824, 42.9752);
    _board_anchor.emplace_back(212.305, 42.3009);
    _board_anchor.emplace_back(398.305, 39.3009);
    _board_anchor.emplace_back(587.469, 38.4179);
    _board_anchor.emplace_back(588.500, 436.5);
    _board_anchor.emplace_back(51.5824, 433.643);
}
bool RTComputer::locateBoard(cv::Mat& img) {
    std::vector<std::vector<Point> > contours;
    std::vector<QuadCountour> contour_quads;
    Mat image_gray,image_bin;
    if(img.channels()==3)
        cvtColor(img,image_gray,COLOR_BGR2GRAY);
    else
        image_gray = img;
    threshold( image_gray, image_bin, 120, 255, THRESH_BINARY );
    generateQuads(image_bin, contour_quads, contours);
    for(auto &contour : contour_quads){
        for(auto &rect : contour.rect) {
            for (int i = 0; i < rect.size(); i++) {
                line(img, rect[i], rect[(i + 1)%rect.size()], Scalar(0, 0, 255), 3, 8);
            }
        }
    }
    _img_anchor.clear();
    if(contour_quads.size()==6) {
        if (sortQuads(contour_quads, _img_anchor)) {
            for (int i = 0; i < _img_anchor.size(); i++) {
                line(img, _img_anchor[i], _img_anchor[(i + 1) % _img_anchor.size()], Scalar(0, 255, 255), 2, 8);
            }
        }
    }
    return !_img_anchor.empty();
}
void RTComputer::clcPointsAndMatches(const cv::Mat& image){
    Mat image_gray;
    if(image.channels()==3)
        cvtColor(image,image_gray,COLOR_BGR2GRAY);
    else
        image_gray = image;
    Mat homography = findHomography(_img_anchor, _board_anchor);
    Mat inv_homo = homography.inv();
    Mat homo_mat;
    warpPerspective(image_gray,homo_mat,homography,Size(_board_width,_board_height));
    //imshow("homo",homo_mat);
    Mat corners_mat;
    vector<Point2f> corners_pro;
    _corner_detector.detectCorners(homo_mat,corners_pro, corners_mat,1);
    FlannBasedMatcher matcher;
    _matches.clear();
    matcher.match(_boardpt_mat, corners_mat, _matches);
    _img_points.clear();
    for (auto&pt:corners_pro) {
        float x = inv_homo.at<double>(0,0)*pt.x+inv_homo.at<double>(0,1)*pt.y+inv_homo.at<double>(0,2);
        float y = inv_homo.at<double>(1,0)*pt.x+inv_homo.at<double>(1,1)*pt.y+inv_homo.at<double>(1,2);
        float z = inv_homo.at<double>(2,0)*pt.x+inv_homo.at<double>(2,1)*pt.y+inv_homo.at<double>(2,2);
        _img_points.emplace_back(x/z,y/z);
    }
    TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 50, 0.001);
    if(!_img_points.empty())
        cornerSubPix(image_gray, _img_points, Size(7, 7), Size(-1, -1), criteria);
}
bool RTComputer::filterMatches() {
    float error = 0;
    vector<Point2f> query,train;
    for(auto& match:_matches) {
        query.push_back(_board_points[match.queryIdx]);
        train.push_back(_img_points[match.trainIdx]);
    }
    Mat F_mat = findFundamentalMat(query,train,FM_RANSAC,0.5);
    for(auto match=_matches.begin();match<_matches.end();match++) {
        Point2f pt1 = _board_points[match->queryIdx];
        Point2f pt2 = _img_points[match->trainIdx];
        float x = F_mat.at<double>(0,0)*pt1.x+F_mat.at<double>(0,1)*pt1.y+F_mat.at<double>(0,2);
        float y = F_mat.at<double>(1,0)*pt1.x+F_mat.at<double>(1,1)*pt1.y+F_mat.at<double>(1,2);
        float z = F_mat.at<double>(2,0)*pt1.x+F_mat.at<double>(2,1)*pt1.y+F_mat.at<double>(2,2);
        float abs_error = abs(pt2.x*x+pt2.y*y+z);
        if(abs_error>1){
            _matches.erase(match);
            match--;
        }else
            error+= abs_error;
    }
    if(_matches.size()<20)
        return false;
    else
        error/=_matches.size();
    return error < 1;
}
bool RTComputer::clcRT(cv::Mat &R, cv::Mat &t) {
    vector<Point2f> query,train;
    Mat cam = (Mat_<double>(3, 3) << 543.15,0,311.96,0,539.49,242.48,0,0,1);
    for(auto& match:_matches) {
        query.push_back(_board_points[match.queryIdx]);
        train.push_back(_img_points[match.trainIdx]);
    }
//    Mat E_mat = findEssentialMat(query,train,cam);
//    int inliers = recoverPose(E_mat,query,train,cam,R,t);
    vector<Point3f> obj;
    for(auto &pt:query){
        obj.emplace_back(pt.x/1000,pt.y/1000,0);
    }
    Mat Rvec;
    Mat dist =(Mat_<double>(1, 5) << -0.00158928,-0.224694,0,0,-0.544474);
    if(!solvePnP(obj,train,cam,dist,Rvec,t))
        return false;
    Rodrigues(Rvec,R);
    return true;
}