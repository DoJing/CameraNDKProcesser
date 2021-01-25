#include <jni.h>
#include <string>

extern "C" JNIEXPORT jstring JNICALL
Java_orb_slam_test_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

extern "C"
JNIEXPORT jbyteArray JNICALL
Java_orb_slam_test_MainActivity_transData(JNIEnv *env, jobject thiz, jbyteArray data, jint len,
                                          jint width, jint height) {
    jbyte gs_raw_data[len];
    jbyte *bytedata = env->GetByteArrayElements(data, 0);
    memset(&gs_raw_data, 0, len);
    memcpy(&gs_raw_data, bytedata, len);

    jbyteArray jarrRV = env->NewByteArray(len);
    env->SetByteArrayRegion(jarrRV, 0, len, gs_raw_data);
    return jarrRV;
}
