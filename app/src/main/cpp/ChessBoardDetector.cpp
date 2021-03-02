//
// Created by dojing on 2021/2/2.
//

#include "ChessBoardDetector.h"

bool filterQuad(std::vector<Point>& contour){
    std::vector<Point> approx_contour;
    const int min_approx_level = 1, max_approx_level = 7;
    for (int approx_level = min_approx_level; approx_level <= max_approx_level; approx_level++ )
    {
        approxPolyDP(contour, approx_contour, (float)approx_level, true);
        if (approx_contour.size() == 4)
            break;
        // we call this again on its own output, because sometimes
        // approxPoly() does not simplify as much as it should.
        std::vector<Point> approx_contour_tmp;
        std::swap(approx_contour, approx_contour_tmp);
        approxPolyDP(approx_contour_tmp, approx_contour, (float)approx_level, true);
        if (approx_contour.size() == 4)
            break;
    }

    // reject non-quadrangles
    if (approx_contour.size() != 4)
        return false;
    if (!cv::isContourConvex(approx_contour))
        return false;


    contour = approx_contour;

    double p = cv::arcLength(approx_contour, true);
    double area = cv::contourArea(approx_contour, false);

    double d1 = sqrt(normL2Sqr<double>(approx_contour[0] - approx_contour[2]));
    double d2 = sqrt(normL2Sqr<double>(approx_contour[1] - approx_contour[3]));

    // philipg.  Only accept those quadrangles which are more square
    // than rectangular and which are big enough
    double d3 = sqrt(normL2Sqr<double>(approx_contour[0] - approx_contour[1]));
    double d4 = sqrt(normL2Sqr<double>(approx_contour[1] - approx_contour[2]));
    if (!(d3*4 > d4 && d4*4 > d3 && d3*d4 < area*1.5 &&
          d1 >= 0.15 * p && d2 >= 0.15 * p))
        return false;
    return true;
}

void generateQuads(const cv::Mat& image_, std::vector<QuadCountour>& contour_quads, std::vector<std::vector<Point> >& contours)
{

//    int g_nStructElementSize = 2;
//    Mat element = getStructuringElement(MORPH_RECT,
//                                        Size(2*g_nStructElementSize+1,2*g_nStructElementSize+1),
//                                        Point( g_nStructElementSize, g_nStructElementSize ));
    //cv::morphologyEx(image_,image_,MORPH_ERODE,element);
    // empiric bound for minimal allowed perimeter for squares
    int min_size = cvRound( image_.cols * image_.rows * .003 * 0.03 );


    std::vector<Vec4i> hierarchy;

    cv::findContours(image_, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    //drawContours(image_,contours,-1,Scalar(128,128,128),2);
    //imshow("contour",image_);
    if (contours.empty())
    {
        return;
    }

    std::vector<int> contour_child_counter(contours.size(), 0);
    //收集最外层轮廓
    std::vector<int> out_contours_idx;
    for (int idx = (int)(contours.size() - 1); idx >= 0; --idx)
    {
        //contour only with two child
        int childIdx = hierarchy[idx][2];
        if(childIdx==-1)
            continue;
        childIdx=hierarchy[childIdx][2];
        if(childIdx==-1)
            continue;
        childIdx=hierarchy[childIdx][2];
        if(childIdx!=-1)
            continue;

        std::vector<Point>& contour = contours[idx];
        Rect contour_rect = boundingRect(contour);
        if (contour_rect.area() < min_size)
            continue;
        if(!filterQuad(contour))
            continue;

        out_contours_idx.push_back(idx);
    }
    for (auto& lv0_idx : out_contours_idx){
        int lv1_idx = hierarchy[lv0_idx][2];
        int lv2_idx = hierarchy[lv1_idx][2];
        std::vector<Point>& lv0_contour = contours[lv0_idx];
        std::vector<Point>& lv1_contour = contours[lv1_idx];
        std::vector<Point>& lv2_contour = contours[lv2_idx];
        Rect lv0_rect = boundingRect(lv0_contour);
        Rect lv1_rect = boundingRect(lv1_contour);
        Rect lv2_rect = boundingRect(lv2_contour);
        if (lv1_rect.area() < min_size
            ||lv2_rect.area() < min_size
            ||lv2_rect.area()<lv1_rect.area()*0.2
            ||lv1_rect.area()<lv0_rect.area()*0.2)
            continue;

        if(!filterQuad(lv1_contour))
            continue;
        if(!filterQuad(lv2_contour))
            continue;
        contour_quads.emplace_back(lv0_contour,lv1_contour,lv2_contour);
    }
    for(auto &contour : contour_quads){
        for(auto &rect : contour.rect) {
            Moments M = moments(rect);
            contour.center.x+=M.m10/M.m00;
            contour.center.y+=M.m01/M.m00;
        }
        contour.center.x/=3;
        contour.center.y/=3;
    }
}
struct Line{
    float A,B,C;
    std::vector<int> point_ids;
    std::vector<int> out_side_ids;
};

bool sortQuads(std::vector<QuadCountour>& contour_quads,std::vector<Point2f>& corner){
    std::vector<Point2f> center;
    corner.resize(6);
    for(auto & countour:contour_quads){
        center.emplace_back(countour.center.x,countour.center.y);
    }
    std::vector<Line> lines;
    float gap = 10;
    for(int i=0;i<center.size()-1;i++){
        for(int j=i+1;j<center.size();j++){
            Line line;
            if(abs(center[i].x-center[j].x)<1) {
                line.A = 1;
                line.B = 0;
                line.C = -(center[i].x + center[j].x) / 2;
            }else{
                line.A = -(center[i].y-center[j].y)/(center[i].x - center[j].x);
                line.B = 1;
                line.C = -line.A*center[i].x-line.B*center[i].y;
            }
            lines.push_back(line);
        }
    }

    for(auto &line : lines){
        for(int id=0;id<center.size();id++){
            Point2f pt = center[id];
            if(abs(pt.x*line.A+pt.y*line.B+line.C)<gap)
                line.point_ids.push_back(id);
            else
                line.out_side_ids.push_back(id);
        }
    }
    for(auto &line:lines){
        if(line.point_ids.size()!=4)
            continue;
        //k较大较小时，误差太大
        if(line.B==0)
            continue;

        int left_right = 0;//1:right,-1:left
        //check all other Point2f on left or right
        for(const auto& id : line.out_side_ids){
            Point2f pt = center[id];
            float cur_d = pt.x*line.A+pt.y*line.B+line.C;
            if(abs(cur_d)<gap)
                continue;
            if(left_right ==0)
                left_right = cur_d>0?1:-1;
            else if(left_right*cur_d<0){//左右异
                left_right=0;
                break;
            }
        }
        if(left_right==0)
            continue;
        int first_id=0,second_id=0;
        float k = -line.A/line.B;
        if(abs(k)>1){
            left_right = k>0?left_right:-left_right;
            if(left_right==-1){
                float min_y=1e10,max_y=-1;
                for(auto &id : line.point_ids){
                    if(center[id].y<min_y){
                        min_y=center[id].y;
                        second_id=id;
                    }
                    if(center[id].y>max_y){
                        max_y=center[id].y;
                        first_id=id;
                    }
                }
            }else{
                float min_y=1e10,max_y=-1;
                for(auto &id : line.point_ids){
                    if(center[id].y<min_y){
                        min_y=center[id].y;
                        first_id=id;
                    }
                    if(center[id].y>max_y){
                        max_y=center[id].y;
                        second_id=id;
                    }
                }
            }
        }else{
            if(left_right==-1){
                float min_x=1e10,max_x=-1;
                for(auto &id : line.point_ids){
                    if(center[id].x<min_x){
                        min_x=center[id].x;
                        second_id=id;
                    }
                    if(center[id].x>max_x){
                        max_x=center[id].x;
                        first_id=id;
                    }
                }
            }else{
                float min_x=1e10,max_x=-1;
                for(auto &id : line.point_ids){
                    if(center[id].x<min_x){
                        min_x=center[id].x;
                        first_id=id;
                    }
                    if(center[id].x>max_x){
                        max_x=center[id].x;
                        second_id=id;
                    }
                }
            }
        }

        contour_quads[first_id].id=0;
        contour_quads[second_id].id=3;
        Point2f first_pt = center[first_id], second_pt = center[second_id];
        corner[0]=Point2f(first_pt.x,first_pt.y);
        corner[3]=Point2f(second_pt.x,second_pt.y);
        for(auto &id : line.point_ids){
            Point2f pt = center[id];
            float dist0=(pt.x-first_pt.x)*(pt.x-first_pt.x)+(pt.y-first_pt.y)*(pt.y-first_pt.y);
            float dist3=(pt.x-second_pt.x)*(pt.x-second_pt.x)+(pt.y-second_pt.y)*(pt.y-second_pt.y);
            if(dist0<1||dist3<1)
                continue;
            if(dist0 < dist3){
                contour_quads[id].id = 1;
                corner[1]=Point2f(pt.x,pt.y);
            } else{
                contour_quads[id].id = 2;
                corner[2]=Point2f(pt.x,pt.y);
            }
        }
        for(auto &id : line.out_side_ids){
            Point2f pt = center[id];
            if(((pt.x-first_pt.x)*(pt.x-first_pt.x)+(pt.y-first_pt.y)*(pt.y-first_pt.y)) <
               ((pt.x-second_pt.x)*(pt.x-second_pt.x)+(pt.y-second_pt.y)*(pt.y-second_pt.y))){
                contour_quads[id].id = 5;
                corner[5]=Point2f(pt.x,pt.y);
            } else{
                contour_quads[id].id = 4;
                corner[4]=Point2f(pt.x,pt.y);
            }
        }
        return true;
    }
    corner.clear();
    return false;
}