#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "yaml-cpp/yaml.h"
#include <Eigen/Dense>

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
int main() {
    //打开摄像头
    cv::VideoCapture capture;
    capture.open(0);
    //灰度图像
    cv::Mat edge;
    //循环显示每一帧
    while (true)
    {
        //frame存储每一帧图像
        cv::Mat frame;
        //读取当前帧
        capture >> frame;
        //显示当前视频
        imshow("正在录制", frame);
        //得到灰度图像
        cvtColor(frame, edge, cv::COLOR_BGR2GRAY);
        //3*3降噪 （2*3+1)
        blur(edge, edge,cv::Size(7,7));
        //边缘显示
        Canny(edge,edge,0,30,3);
        imshow("高斯模糊视频",edge);
    }
    return 0;
}
