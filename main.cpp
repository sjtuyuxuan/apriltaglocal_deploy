#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "yaml-cpp/yaml.h"
#include <Eigen/Dense>
#include <time.h>
#include <iostream>
extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
}

#include <deque>
#include <time.h>
#include <mutex>
#include <thread>


inline void ToArray (const Eigen::Affine3d& pose, double* T_out) {
  Eigen::Quaterniond q_eigen_out(pose.rotation());
  T_out[0] = q_eigen_out.w();
  T_out[1] = q_eigen_out.x();
  T_out[2] = q_eigen_out.y();
  T_out[3] = q_eigen_out.z();
  T_out[4] = pose.translation().x();
  T_out[5] = pose.translation().y();
  T_out[6] = pose.translation().z();
}

inline Eigen::Affine3d ToEigen (double* T_in) {
  Eigen::Affine3d rtn =
      Eigen::Translation3d(T_in[4], T_in[5], T_in[6]) *
      Eigen::Quaterniond(T_in[0], T_in[1], T_in[2], T_in[3]);
  return rtn;
}

inline void ArrayProduct (double* T_0, double* T_1, double* T_out) {
  ToArray(ToEigen(T_0) * ToEigen(T_1), T_out);
}


struct CostFunctor {
  CostFunctor(Eigen::Affine3d Tab) : Tab_(Tab){}
  template <typename T>
  bool operator()(const T* const qa, const T* const ta, const T* const qb, const T* const tb, T* residual) const {
    T qab[4];
    T qb_calculated[4];
    T q_diff[4];
    Eigen::Quaterniond qab_eigen(Tab_.rotation());
    qab[0] = T(qab_eigen.w());
    qab[1] = T(qab_eigen.x());
    qab[2] = T(qab_eigen.y());
    qab[3] = T(qab_eigen.z());
    ceres::QuaternionProduct(qa, qab, qb_calculated);
    qb_calculated[0] = -qb_calculated[0];
    ceres::QuaternionProduct(qb, qb_calculated, q_diff);
    residual[0] = q_diff[1];
    residual[1] = q_diff[2];
    residual[2] = q_diff[3];
    
    T t_w_diff[3];
    T t_a_diff[3];
    t_a_diff[0] = T(Tab_.translation().x());
    t_a_diff[1] = T(Tab_.translation().y());
    t_a_diff[2] = T(Tab_.translation().z());
    ceres::QuaternionRotatePoint(qa, t_a_diff, t_w_diff);
    residual[3] = -t_w_diff[0] + tb[0] - ta[0];;
    residual[4] = -t_w_diff[1] + tb[1] - ta[1];;
    residual[5] = -t_w_diff[2] + tb[2] - ta[2];;
    return true;
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  Eigen::Affine3d Tab_;
};

void Backend (ceres::Problem& problem,
              std::vector<std::vector<double>>& T_wtag,
              std::set<int>& valid,
              bool& new_flag,
              std::map<int, Eigen::Affine3d, std::less<int>,
                  Eigen::aligned_allocator<std::pair<const int, Eigen::Affine3d>>>& T_tag_cam,
              std::mutex& T_tag_cam_lock) {
  std::map<std::pair<int, int>, std::deque<ceres::ResidualBlockId>> ids;
  while (true) {
    if (!new_flag || T_tag_cam.size() < 2) continue;
    T_tag_cam_lock.lock();
    Eigen::Affine3d base = T_tag_cam.begin()->second;
    int base_id = T_tag_cam.begin()->first;
    for (auto it = ++T_tag_cam.begin(); it != T_tag_cam.end(); ++it) {
      if (it->first != 0 && T_wtag[it->first][0] == 1) {
        ToArray(ToEigen(T_wtag[base_id].data()) * base * it->second.inverse(), T_wtag[it->first].data());
      } else if (base_id != 0 && T_wtag[base_id][0] == 1) {
        ToArray(ToEigen(T_wtag[it->first].data()) * it->second * base.inverse(), T_wtag[base_id].data());
      }
      auto cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 6, 4, 3, 4, 3>(new CostFunctor(base * it->second.inverse()));
      auto id = problem.AddResidualBlock(cost_function,
          NULL,
          T_wtag[base_id].data(),
          T_wtag[base_id].data()+4,
          T_wtag[it->first].data(),
          T_wtag[it->first].data()+4);
      if (ids.find(std::pair<int, int>(base_id, it->first)) == ids.end()) {
        ids[std::pair<int, int>(base_id, it->first)] = {id};
      } else if (ids[std::pair<int, int>(base_id, it->first)].size() < 50) {
        ids[std::pair<int, int>(base_id, it->first)].emplace_back(id);
      } else {
        ids[std::pair<int, int>(base_id, it->first)].emplace_back(id);
        problem.RemoveResidualBlock(ids[std::pair<int, int>(base_id, it->first)].front());
        ids[std::pair<int, int>(base_id, it->first)].pop_front();
      }
    }
    std::vector<int> temp{};
    bool term_valid = false;
    for (auto& a : T_tag_cam) temp.emplace_back(a.first);
    for (auto& a : temp) {
      if (valid.find(a) != valid.end()) {
        term_valid = true;
        break;
      }
    }
    T_tag_cam_lock.unlock();
    new_flag = false;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (term_valid) for (auto& a : temp) valid.insert(a);
  }
}

int main(int argc, char** argv) {
  cv::VideoCapture capture;
  if (argc != 2) {
    std::cout << "plz set yaml file path" << std::endl;
    return -1;
  }

  std::string config_file_path_(argv[1]); 
  std::cout << "yaml file path is:" << config_file_path_ << std::endl;
  YAML::Node config_ = YAML::LoadFile(config_file_path_);

  capture.open("/dev/video0", 0);
  capture.set(cv::CAP_PROP_FRAME_WIDTH,640);
  capture.set(cv::CAP_PROP_FRAME_HEIGHT,480);
  std::cout << "Cam /dev/video0 is Opened and set resolution to 640 * 480" << std::endl;  

  cv::Mat distCoeffs(5,1,cv::DataType<double>::type);
  distCoeffs.at<double>(0) = config_["ImageDistrotion"][0].as<double>(0);
  distCoeffs.at<double>(1) = config_["ImageDistrotion"][1].as<double>(0);
  distCoeffs.at<double>(2) = config_["ImageDistrotion"][2].as<double>(0);
  distCoeffs.at<double>(3) = config_["ImageDistrotion"][3].as<double>(0);
  distCoeffs.at<double>(4) = config_["ImageDistrotion"][4].as<double>(0);

  cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
  cameraMatrix.at<double>(0, 0) = config_["ImageIntrinsic"][0].as<double>(0);
  cameraMatrix.at<double>(0, 1) = config_["ImageIntrinsic"][1].as<double>(0);
  cameraMatrix.at<double>(0, 2) = config_["ImageIntrinsic"][2].as<double>(0);
  cameraMatrix.at<double>(1, 0) = config_["ImageIntrinsic"][3].as<double>(0);
  cameraMatrix.at<double>(1, 1) = config_["ImageIntrinsic"][4].as<double>(0);
  cameraMatrix.at<double>(1, 2) = config_["ImageIntrinsic"][5].as<double>(0);
  cameraMatrix.at<double>(2, 0) = config_["ImageIntrinsic"][6].as<double>(0);
  cameraMatrix.at<double>(2, 1) = config_["ImageIntrinsic"][7].as<double>(0);
  cameraMatrix.at<double>(2, 2) = config_["ImageIntrinsic"][8].as<double>(0);

  apriltag_family_t *tf = NULL;
  apriltag_detector_t *td = apriltag_detector_create();
  std::string famname = config_["TagFamily"].as<std::string>("tag16h5");
  if (famname == "tag36h11") {
      tf = tag36h11_create();
  } else if (famname ==  "tag25h9") {
      tf = tag25h9_create();
  } else if (famname == "tag16h5") {
      tf = tag16h5_create();
  } else if (famname == "tagCircle21h7") {
      tf = tagCircle21h7_create();
  } else if (famname == "tagCircle49h12") {
      tf = tagCircle49h12_create();
  } else if (famname == "tagStandard41h12") {
      tf = tagStandard41h12_create();
  } else if (famname == "tagStandard52h13") {
      tf = tagStandard52h13_create();
  } else if (famname == "tagCustom48h12") {
      tf = tagCustom48h12_create();
  }
  apriltag_detector_add_family(td, tf);
  td->debug = 0;
  td->quad_decimate = 2;
  td->refine_edges = 0;
  td->quad_sigma = 0;
  td->nthreads = 4;

  std::set<int> tag_enum;
  for (int id : config_["GenTags"].as<std::vector<int>>()) tag_enum.insert(id);

  ceres::Problem problem;
  std::vector<std::vector<double>> T_wtag(30, std::vector<double>(7, 0));
  for (int i = 0; i < 30; ++i) {
    T_wtag[i][0] = 1;
    problem.AddParameterBlock(T_wtag[i].data(), 4);
    problem.AddParameterBlock(T_wtag[i].data()+4, 3);
    problem.SetParameterization(T_wtag[i].data(), new ceres::QuaternionParameterization());
  }
  problem.SetParameterBlockConstant(T_wtag[0].data());
  problem.SetParameterBlockConstant(T_wtag[0].data()+4);

  std::vector<double> T_tag0_cam(7, 0);
  // T_tag0_cam[0] = 1;
  // problem.AddParameterBlock(T_tag0_cam.data(), 4);
  // problem.AddParameterBlock(T_tag0_cam.data()+4, 3);
  // problem.SetParameterization(T_tag0_cam.data(), new ceres::QuaternionParameterization());
  std::map<int, Eigen::Affine3d, std::less<int>,
        Eigen::aligned_allocator<std::pair<const int, Eigen::Affine3d>>> T_tag_cam_pipe;
  bool new_flag = false;
  std::mutex T_tag_cam_lock;
  std::set<int> valid {0};
  std::thread t(Backend, std::ref(problem),
      std::ref(T_wtag),
      std::ref(valid),
      std::ref(new_flag),
      std::ref(T_tag_cam_pipe),
      std::ref(T_tag_cam_lock));
  t.detach();

  cv::Mat gray_img;
  int i = 0;
  while (capture.isOpened())
  {   
    ++i;
    cv::Mat rgb_img;
    capture >> rgb_img;
    cvtColor(rgb_img, gray_img, cv::COLOR_BGR2GRAY);
    image_u8_t im = {
          .width = gray_img.cols,
          .height = gray_img.rows,
          .stride = gray_img.cols,
          .buf = gray_img.data};
    zarray_t *detections = apriltag_detector_detect(td, &im);
    std::map<int, Eigen::Affine3d, std::less<int>,
        Eigen::aligned_allocator<std::pair<const int, Eigen::Affine3d>>> T_tag_cam;
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      Eigen::Vector2d center(det->c[0], det->c[1]);
      Eigen::Vector2d ab(det->p[1][0] - det->p[0][0], det->p[1][1] - det->p[0][1]);
      Eigen::Vector2d bc(det->p[2][0] - det->p[1][0], det->p[2][1] - det->p[1][1]);
      double ratio = ab.norm() / bc.norm();
      double angle_cos = std::abs(ab.dot(bc)) / (ab.norm() * bc.norm());
      if (ratio > 0.66 && ratio < 1.5 && angle_cos < 0.5 && tag_enum.find(det->id) != tag_enum.end() && det->hamming == 0) {
        auto dots = config_[det->id]["Extand_Dot"].as<std::vector<std::vector<double>>>();
        auto tag_size = config_[det->id]["Size"].as<double>();
        std::vector<cv::Point3d> tag_points;
        std::vector<cv::Point2d> image_points;
        image_points.emplace_back(cv::Point2d(det->p[0][0], det->p[0][1]));
        image_points.emplace_back(cv::Point2d(det->p[1][0], det->p[1][1]));
        image_points.emplace_back(cv::Point2d(det->p[2][0], det->p[2][1]));
        image_points.emplace_back(cv::Point2d(det->p[3][0], det->p[3][1]));
        tag_points.emplace_back(cv::Point3d(-tag_size/2.0, -tag_size/2.0, 0));
        tag_points.emplace_back(cv::Point3d(tag_size/2.0, -tag_size/2.0, 0));
        tag_points.emplace_back(cv::Point3d(tag_size/2.0, tag_size/2.0, 0));
        tag_points.emplace_back(cv::Point3d(-tag_size/2.0, tag_size/2.0, 0));
        for (const auto& dot : dots) {
          Eigen::Vector2i dot_center;
          dot_center.x() = center.x() + ab.x() / tag_size * dot[0] + bc.x() / tag_size * dot[1];
          dot_center.y() = center.y() + ab.y() / tag_size * dot[0] + bc.y() / tag_size * dot[1];
          int dot_size = 1.3 * (ab.norm() + bc.norm()) / tag_size * dot[2];
          if (dot_center.x() - dot_size < 0 || dot_center.y() - dot_size < 0 ||
              dot_center.x() + dot_size > gray_img.cols || dot_center.y() + dot_size > gray_img.rows) continue;
          cv::Rect rect(dot_center.x() - dot_size, dot_center.y() - dot_size, dot_size * 2, dot_size * 2);
          cv::Mat dot_pat = gray_img(rect);
          cv::Mat_<uchar> dot_pat_uchar = dot_pat;
          double mean = cv::mean(dot_pat).val[0];
          double center_x = 0;
          double center_y = 0;
          int count = 0;
          for (int i = 0; i < 2 * dot_size; ++ i)
            for (int j = 0; j < 2 * dot_size; ++ j)
              if (dot_pat_uchar(i, j) < mean/1.2) {
                center_x += j;
                center_y += i;
                count += 1;
              }
          center_x /= count;
          center_y /= count;
          image_points.emplace_back(cv::Point2d(dot_center.x() - dot_size + center_x, dot_center.y() - dot_size + center_y));
          tag_points.emplace_back(cv::Point3d(dot[0], dot[1], 0));
          cv::circle(gray_img, cv::Point(dot_center.x() - dot_size + center_x, dot_center.y() - dot_size + center_y), 4, cv::Scalar(0), -1);
          cv::circle(gray_img, cv::Point(dot_center.x(), dot_center.y()), dot_size, cv::Scalar(0x4f), 2);
        }
        cv::line(gray_img, cv::Point(det->p[0][0], det->p[0][1]),
                  cv::Point(det->p[1][0], det->p[1][1]),
                  cv::Scalar(0), 2);
        cv::line(gray_img, cv::Point(det->p[0][0], det->p[0][1]),
                  cv::Point(det->p[3][0], det->p[3][1]),
                  cv::Scalar(0x2f), 2);
        cv::line(gray_img, cv::Point(det->p[1][0], det->p[1][1]),
                  cv::Point(det->p[2][0], det->p[2][1]),
                  cv::Scalar(0x5f), 2);
        cv::line(gray_img, cv::Point(det->p[2][0], det->p[2][1]),
                  cv::Point(det->p[3][0], det->p[3][1]),
                  cv::Scalar(0x5f), 2);
        cv::Mat rvec(3,1,cv::DataType<double>::type);
        cv::Mat tvec(3,1,cv::DataType<double>::type);
        cv::solvePnP(tag_points, image_points, cameraMatrix, distCoeffs, rvec, tvec);
        Eigen::Vector3d rot_vec(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
        Eigen::Vector3d trans(tvec.at<double>(0) / 1000.0, tvec.at<double>(1) / 1000.0, tvec.at<double>(2) / 1000.0);
        Eigen::Matrix3d rot(Eigen::AngleAxisd(rot_vec.norm(), rot_vec.normalized()));
        Eigen::Affine3d T_cw = Eigen::Translation3d(trans) * rot;
        Eigen::Affine3d T_wc = T_cw.inverse();
        //std::cout << "t_wc: \n" << T_wc.translation() << std::endl;
        if (T_wc.translation()[2] < 50000) {
          T_tag_cam[det->id] = T_wc;
        }
      } 
    }
    if (T_tag_cam.empty()) {
      cv::imshow("test", gray_img);
      cv::waitKey(1);
      continue;
    }
    
    T_tag_cam_lock.lock();
    T_tag_cam_pipe = T_tag_cam;
    new_flag = true;
    T_tag_cam_lock.unlock();

    std::vector<std::vector<double>> T_w_cam{};
    for (auto& a : T_tag_cam) {
      if (valid.find(a.first) != valid.end()) {
        T_w_cam.emplace_back(std::vector<double>(7, 0));
        Eigen::Affine3d base_pose = ToEigen(T_wtag[a.first].data());
        ToArray(base_pose * a.second, T_w_cam.back().data());
      }
    }
    if (!T_w_cam.empty()) {
      T_tag0_cam = T_w_cam.front();
      T_tag0_cam[6] = T_tag0_cam[5] = T_tag0_cam[4] = 0;
      for (auto T : T_w_cam) {
        T_tag0_cam[4] += T[4] / int(T_w_cam.size());
        T_tag0_cam[5] += T[5] / int(T_w_cam.size());
        T_tag0_cam[6] += T[6] / int(T_w_cam.size());
      }
      std::cout << i << "farme position (xyz):";
      std::cout << T_tag0_cam[4] << "   " 
                << T_tag0_cam[5] << "   " 
                << T_tag0_cam[6] << std::endl;
      cv::putText(gray_img, "Farme position (xyz):", cv::Point2i(20, 20),
          cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0), 2);
      cv::putText(gray_img, std::to_string(T_tag0_cam[4]), cv::Point2i(20, 50),
          cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0), 2);
      cv::putText(gray_img, std::to_string(T_tag0_cam[5]), cv::Point2i(20, 80),
          cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0), 2);
      cv::putText(gray_img, std::to_string(T_tag0_cam[6]), cv::Point2i(20, 110),
          cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0), 2);
    }


    // blur(edge, edge,cv::Size(7,7));
    // Canny(edge,edge,0,30,3);
    cv::imshow("test", gray_img);
    cv::waitKey(1);
  }
  return 0;
}
