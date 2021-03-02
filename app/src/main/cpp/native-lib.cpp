#include <jni.h>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "PointDetector.h"
#include "RTComputer.h"
#include <android/log.h>
#define TAG "projectname" // 这个是自定义的LOG的标识
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,TAG ,__VA_ARGS__) // 定义LOGD类型
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,TAG ,__VA_ARGS__) // 定义LOGI类型
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,TAG ,__VA_ARGS__) // 定义LOGW类型
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,TAG ,__VA_ARGS__) // 定义LOGE类型
#define LOGF(...) __android_log_print(ANDROID_LOG_FATAL,TAG ,__VA_ARGS__) // 定义LOGF类型

extern "C" JNIEXPORT jstring JNICALL
Java_orb_slam_test_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

bool RTDetect(cv::Mat& image_gray,Mat &R,Mat& t){

    CornerDetector corner_detector;
    RTComputer computer;
    computer.initTemplatePoints();

    if(computer.locateBoard(image_gray)){
        computer.clcPointsAndMatches(image_gray);
        if(computer.filterMatches()){
            if(computer.clcRT(R,t)) {
                return true;
            }
        }
    }
    return false;
}



extern "C"
JNIEXPORT jboolean JNICALL
Java_orb_slam_test_MainActivity_transData(JNIEnv *env, jobject thiz, jbyteArray img, jint len,
                                          jint width, jint height,jdoubleArray R_arr,jdoubleArray t_arr) {
    jbyte *imgdata = env->GetByteArrayElements(img, 0);
    jdouble *R_data = env->GetDoubleArrayElements(R_arr, 0);
    jdouble *t_data = env->GetDoubleArrayElements(t_arr, 0);
    cv::Mat img_Y(height,width,CV_8UC1,imgdata);
    cv::Mat img_UV(height/2,width,CV_8UC1,imgdata+height*width);
    cv::Mat img_UV_r(width/2,height,CV_8UC1);
    cv::rotate(img_Y, img_Y, 0);

    for(int r=0;r<img_UV_r.rows;r++){
        uchar* uv_r_ptr=img_UV_r.ptr<uchar>(r);
        for(int c=0;c<img_UV_r.cols;c+=2){
            uv_r_ptr[img_UV_r.cols -1 - c + 1]=img_UV.at<uchar>(c/2,2*r);
            uv_r_ptr[img_UV_r.cols -1 - c ]=img_UV.at<uchar>(c/2,2*r+1);
        }
    }
    bool success = false;
    Mat R(3,3,CV_64FC1,R_data),t(3,1,CV_64FC1,t_data);
    if(RTDetect(img_Y, R,t)){
        std::stringstream ss;
        for(int r=0;r<3;r++){
            for(int c=0;c<3;c++){
                ss<<R.at<double>(r,c)<<",";
            }
        }
        char* info = (char*)ss.str().c_str();
        LOGI("%s\n",info);
        memcpy(R_data,R.data,8*9);
        memcpy(t_data,t.data,8*3);
        success = true;
    }
    memcpy(imgdata,img_Y.data,height*width);
    memcpy(imgdata+height*width,img_UV_r.data,height*width/2);
    return success;
}


extern "C"
JNIEXPORT jobject JNICALL
Java_orb_slam_test_MainActivity_transBitmap(JNIEnv *env, jobject thiz, jbyteArray data, jint len,
                                            jint width, jint height) {
    // TODO: implement transBitmap()
}