// JNI for com.scan3d.arlat.ArLoop (one LoopCloser per handle).
#include <jni.h>

#include <opencv2/core.hpp>

#include "arloop.h"

using arloop::LoopCloser;

extern "C" {

JNIEXPORT jlong JNICALL Java_com_scan3d_arlat_ArLoop_nCreate(JNIEnv *, jclass, jboolean markers, jboolean features,
                                                             jint dict, jfloat marker_len) {
  arloop::Options o;
  o.markers = markers;
  o.features = features;
  o.marker_dict = dict;
  o.marker_len = marker_len;
  return reinterpret_cast<jlong>(new LoopCloser(o));
}

JNIEXPORT void JNICALL Java_com_scan3d_arlat_ArLoop_nDestroy(JNIEnv *, jclass, jlong h) {
  delete reinterpret_cast<LoopCloser *>(h);
}

// pose = {tx, ty, tz, qx, qy, qz, qw} (ARCore Camera.getPose()), K = {fx, fy, cx, cy}, depth may be null
JNIEXPORT void JNICALL Java_com_scan3d_arlat_ArLoop_nAddKeyframe(JNIEnv *env, jclass, jlong h, jlong t_ns,
                                                                 jfloatArray pose, jbyteArray gray, jint w, jint hh,
                                                                 jfloatArray K, jshortArray depth, jint dw, jint dh) {
  arloop::KeyframeIn kf;
  kf.t_ns = t_ns;
  jfloat p[7], k[4];
  env->GetFloatArrayRegion(pose, 0, 7, p);
  env->GetFloatArrayRegion(K, 0, 4, k);
  kf.t_gl = Eigen::Vector3d(p[0], p[1], p[2]);
  kf.q_gl = Eigen::Quaterniond(p[6], p[3], p[4], p[5]);
  kf.fx = k[0]; kf.fy = k[1]; kf.cx = k[2]; kf.cy = k[3];
  kf.gray.create(hh, w, CV_8UC1);
  env->GetByteArrayRegion(gray, 0, w * hh, reinterpret_cast<jbyte *>(kf.gray.data));
  if (depth != nullptr && dw > 0 && dh > 0) {
    kf.depth_mm.create(dh, dw, CV_16UC1);
    env->GetShortArrayRegion(depth, 0, dw * dh, reinterpret_cast<jshort *>(kf.depth_mm.data));
  }
  reinterpret_cast<LoopCloser *>(h)->add_keyframe(std::move(kf));
}

// in = ARCore pose (7), out = corrected pose (7)
JNIEXPORT void JNICALL Java_com_scan3d_arlat_ArLoop_nCorrect(JNIEnv *env, jclass, jlong h, jlong t_ns,
                                                             jfloatArray in, jfloatArray out) {
  jfloat p[7];
  env->GetFloatArrayRegion(in, 0, 7, p);
  Eigen::Vector3d t;
  Eigen::Quaterniond q;
  reinterpret_cast<LoopCloser *>(h)->correct(t_ns, Eigen::Vector3d(p[0], p[1], p[2]),
                                             Eigen::Quaterniond(p[6], p[3], p[4], p[5]), t, q);
  jfloat o[7] = {float(t.x()), float(t.y()), float(t.z()), float(q.x()), float(q.y()), float(q.z()), float(q.w())};
  env->SetFloatArrayRegion(out, 0, 7, o);
}

JNIEXPORT jstring JNICALL Java_com_scan3d_arlat_ArLoop_nStats(JNIEnv *env, jclass, jlong h) {
  return env->NewStringUTF(reinterpret_cast<LoopCloser *>(h)->stats_json().c_str());
}

}  // extern "C"
