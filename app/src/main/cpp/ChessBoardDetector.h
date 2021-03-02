#ifndef QRCODEDETECTOR_CHESSBOARDDETECTOR_H
#define QRCODEDETECTOR_CHESSBOARDDETECTOR_H

#include <opencv2/opencv.hpp>
#include <utility>
#include <numeric>

using namespace cv;
struct QuadCountour {
    std::vector<Point> rect[3];
    Point2f center;
    int id;
    QuadCountour(std::vector<Point> lv0,std::vector<Point> lv1,std::vector<Point> lv2){
        rect[0]=std::move(lv0);
        rect[1]=std::move(lv1);
        rect[2]=std::move(lv2);
        center=Point2f(0,0);
        id=-1;
    }
};
bool filterQuad(std::vector<Point>& contour);
void generateQuads(const cv::Mat& image_, std::vector<QuadCountour>& contour_quads, std::vector<std::vector<Point> >& contours);
bool sortQuads(std::vector<QuadCountour>& contour_quads,std::vector<Point2f>& corner);
#endif