/*  Copyright 2017 onlyliu(997737609@qq.com).                                */
/*                                                                        */
/*  part of source code come from https://github.com/qibao77/cornerDetect */
/*  Automatic Camera and Range Sensor Calibration using a single Shot     */
/*  this project realize the papar: Automatic Camera and Range Sensor     */
/*  Calibration using a single Shot                                       */


#include "PointDetector.h"

#include <utility>

using namespace cv;
using namespace std;

CornerDetector::~CornerDetector()
{
}
CornerDetector::CornerDetector()
{
    createkernel(0, CV_PI / 2, 8, kernelA1, kernelB1, kernelC1, kernelD1);
}

dtype CornerDetector::normpdf(dtype dist, dtype mu, dtype sigma)
{
    dtype s = exp(-0.5*(dist - mu)*(dist - mu) / (sigma*sigma));
    s = s / (std::sqrt(2 * CV_PI)*sigma);
    return s;
}
void CornerDetector::createkernel(float angle1, float angle2, int kernelSize, Mat &kernelA, Mat &kernelB, Mat &kernelC, Mat &kernelD)
{

	int width = (int)kernelSize * 2 + 1;
	int height = (int)kernelSize * 2 + 1;
	kernelA = cv::Mat::zeros(height, width, mtype);
	kernelB = cv::Mat::zeros(height, width, mtype);
	kernelC = cv::Mat::zeros(height, width, mtype);
	kernelD = cv::Mat::zeros(height, width, mtype);

	for (int u = 0; u < width; ++u){
		for (int v = 0; v < height; ++v){
			dtype vec[] = { static_cast<float>(u - kernelSize), static_cast<float>(v - kernelSize) };
			dtype dis = std::sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
			dtype side1 = vec[0] * (-sin(angle1)) + vec[1] * cos(angle1);
			dtype side2 = vec[0] * (-sin(angle2)) + vec[1] * cos(angle2);//X=X0*cos+Y0*sin;Y=Y0*cos-X0*sin
			if (side1 <= -0.1&&side2 <= -0.1){
				kernelA.ptr<dtype>(v)[u] = normpdf(dis, 0, kernelSize / 2);
			}
			if (side1 >= 0.1&&side2 >= 0.1){
				kernelB.ptr<dtype>(v)[u] = normpdf(dis, 0, kernelSize / 2);
			}
			if (side1 <= -0.1&&side2 >= 0.1){
				kernelC.ptr<dtype>(v)[u] = normpdf(dis, 0, kernelSize / 2);
			}
			if (side1 >= 0.1&&side2 <= -0.1){
				kernelD.ptr<dtype>(v)[u] = normpdf(dis, 0, kernelSize / 2);
			}
		}
	}
	kernelA = kernelA / cv::sum(kernelA)[0];
	kernelB = kernelB / cv::sum(kernelB)[0];
	kernelC = kernelC / cv::sum(kernelC)[0];
	kernelD = kernelD / cv::sum(kernelD)[0];
}


//*************************************************************************//
void CornerDetector::getMin(Mat src1, Mat src2, Mat &dst){
	int rowsLeft = src1.rows;
	int colsLeft = src1.cols;
	int rowsRight = src2.rows;
	int colsRight = src2.cols;
	if (rowsLeft != rowsRight || colsLeft != colsRight)return;

	int channels = src1.channels();

	int nr = rowsLeft;
	int nc = colsLeft;
	if (src1.isContinuous()){
		nc = nc*nr;
		nr = 1;
	}
	for (int i = 0; i < nr; i++){
		const dtype* dataLeft = src1.ptr<dtype>(i);
		const dtype* dataRight = src2.ptr<dtype>(i);
		dtype* dataResult = dst.ptr<dtype>(i);
		for (int j = 0; j < nc*channels; ++j){
			dataResult[j] = (dataLeft[j] < dataRight[j]) ? dataLeft[j] : dataRight[j];
		}
	}
}
//*************************************************************************//
void CornerDetector::getMax(Mat src1, Mat src2, Mat &dst)
{
	int rowsLeft = src1.rows;
	int colsLeft = src1.cols;
	int rowsRight = src2.rows;
	int colsRight = src2.cols;
	if (rowsLeft != rowsRight || colsLeft != colsRight)return;

	int channels = src1.channels();

	int nr = rowsLeft;
	int nc = colsLeft;
	if (src1.isContinuous()){
		nc = nc*nr;
		nr = 1;
		//std::cout<<"continue"<<std::endl;
	}
	for (int i = 0; i < nr; i++){
		const dtype* dataLeft = src1.ptr<dtype>(i);
		const dtype* dataRight = src2.ptr<dtype>(i);
		dtype* dataResult = dst.ptr<dtype>(i);
		for (int j = 0; j < nc*channels; ++j){
			dataResult[j] = (dataLeft[j] >= dataRight[j]) ? dataLeft[j] : dataRight[j];
		}
	}
}


void CornerDetector::nonMaximumSuppression(Mat& inputCorners, vector<Point2f>& outputCorners, int patchSize, dtype threshold, int margin)
{
	if (inputCorners.empty())
	{
		cout << "The imput mat is empty!" << endl; return;
	}
	for (int i = margin + patchSize; i <= inputCorners.cols - (margin + patchSize+1); i = i + patchSize + 1)
	{
		for (int j = margin + patchSize; j <= inputCorners.rows - (margin + patchSize+1); j = j + patchSize + 1)
		{
			dtype maxVal = inputCorners.ptr<dtype>(j)[i];
			int maxX = i; int maxY = j;
			for (int m = i; m <= i + patchSize ; m++)
			{
				for (int n = j; n <= j + patchSize ; n++)
				{
					dtype temp = inputCorners.ptr<dtype>(n)[m];
					if (temp > maxVal)
					{
						maxVal = temp; maxX = m; maxY = n;
					}
				}
			}
			if (maxVal < threshold)continue;
			int flag = 0;
			for (int m = maxX - patchSize; m <= min(maxX + patchSize, inputCorners.cols - margin-1); m++)//???¦Ì??
			{
				for (int n = maxY - patchSize; n <= min(maxY + patchSize, inputCorners.rows - margin-1); n++)
				{
					if (inputCorners.ptr<dtype>(n)[m]>maxVal && (m<i || m>i + patchSize || n<j || n>j + patchSize))
					{
						flag = 1; break;
					}
				}
				if (flag)break;
			}
			if (flag)continue;
			outputCorners.push_back(Point(maxX, maxY));
		}
	}
}

void CornerDetector::detectCorners(const cv::Mat& Src, std::vector<cv::Point2f>& outputCorners, bool isrefine)
{
    Mat gray, imageNorm;
    // convert to double grayscale image
    if (Src.channels() == 3)
        cvtColor(Src, gray, COLOR_BGR2GRAY);
    else
        gray = Src;

    Mat mask = Mat::zeros(gray.size(),gray.type());
    rectangle(mask,Point(mask.cols*0.2,mask.rows*0.2),Point(mask.cols*0.8,mask.rows*0.8),Scalar(1),-1);
    gray = gray.mul(mask);

    // scale input image
    normalize(gray, imageNorm, 0, 1, cv::NORM_MINMAX, mtype);

    // filter image
    Mat imgCorners = Mat::zeros(imageNorm.size(), mtype);

    Mat imgCornerA1(imageNorm.size(), mtype);
    Mat imgCornerB1(imageNorm.size(), mtype);
    Mat imgCornerC1(imageNorm.size(), mtype);
    Mat imgCornerD1(imageNorm.size(), mtype);

    Mat imgCornerA(imageNorm.size(), mtype);
    Mat imgCornerB(imageNorm.size(), mtype);
    Mat imgCorner1(imageNorm.size(), mtype);
    Mat imgCorner2(imageNorm.size(), mtype);
    Mat imgCornerMean(imageNorm.size(), mtype);

    filter2D(imageNorm, imgCornerA1, mtype, kernelA1);//a1
    filter2D(imageNorm, imgCornerB1, mtype, kernelB1);//a2
    filter2D(imageNorm, imgCornerC1, mtype, kernelC1);//b1
    filter2D(imageNorm, imgCornerD1, mtype, kernelD1);//b2
    //compute mean
    imgCornerMean = (imgCornerA1 + imgCornerB1 + imgCornerC1 + imgCornerD1) / 4.0;//1.3 ?????????§Þ???
    // case 1: a = white, b = black
    getMin(imgCornerA1 - imgCornerMean, imgCornerB1 - imgCornerMean, imgCornerA);
    getMin(imgCornerMean - imgCornerC1, imgCornerMean - imgCornerD1, imgCornerB);
    getMin(imgCornerA, imgCornerB, imgCorner1);
    // case 2: b = white, a = black
    getMin(imgCornerMean - imgCornerA1, imgCornerMean - imgCornerB1, imgCornerA);
    getMin(imgCornerC1 - imgCornerMean, imgCornerD1 - imgCornerMean, imgCornerB);
    getMin(imgCornerA, imgCornerB, imgCorner2);

    // update corner map
    getMax(imgCorners, imgCorner1, imgCorners);
    getMax(imgCorners, imgCorner2, imgCorners);

    // extract corner candidates via non maximum suppression
    nonMaximumSuppression(imgCorners, outputCorners, 3, 0.025, 5);


    //post processing
    if (isrefine)
    {
        TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 50, 0.001);
        if(!outputCorners.empty())
            cornerSubPix(gray, outputCorners, Size(7, 7), Size(-1, -1), criteria);
    }
}

void CornerDetector::detectCorners(const cv::Mat& Src, std::vector<cv::Point2f>& corners_vec, Mat& corners_mat, bool isrefine){
    detectCorners(Src,corners_vec,isrefine);
    corners_mat = Mat(corners_vec.size(),2,CV_32FC1);
    for(int i=0;i<corners_vec.size();i++) {
        corners_mat.at<float>(i,0)=corners_vec[i].x;
        corners_mat.at<float>(i,1)=corners_vec[i].y;
    }
}