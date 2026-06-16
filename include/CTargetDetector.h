#ifndef __CTARGETDETECTOR_H__
#define __CTARGETDETECTOR_H__

#include <iostream>
#include <string.h>
#include <algorithm>
#include <omp.h>
#include <sys/time.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include "CCircleGridFinder.hpp"
#include "utils.h"


using namespace std;

typedef pair<Shape,std::vector<cv::Point>> circle_info;

class WrongTypeException: public std::exception{
    public:
        const char* what(){
            return "Type is wrong";
        }
};

class TargetDetector{
    public:
        /// @brief 检测圆
        /// @param n_x 宽度
        /// @param n_y 高度
        /// @param draw 输出draw
        TargetDetector(int n_x, int n_y, bool draw = true);
        /// @brief 入口函数 检测椭圆
        /// @param img 
        /// @param type 标定板类型 square circle
        /// @return 检测结果bool是否成功，shape
        pair<bool,vector<Shape>>detect(cv::Mat& img, string type);
        /// @brief 预处理 单通道双边滤波 3通道转单通道滤波
        /// @param img 
        /// @param detection_mode saturation时转到hsv 取s通道,
        /// @return
        static cv::Mat preprocessing(cv::Mat img, string detection_mode);
        static cv::Mat translation_blur(const cv::Mat &img, double trans);
        static cv::Mat rotation_blur(const cv::Mat &img, double dtheta);
        void save_result(string path);

    private:
        double numerical_stable;
        bool draw;
        cv::Mat detection_result;
        int n_x, n_y;
        int size_threshold;
        float drawing_scale;
        double fullfill_threshold, eccentricity_threshold, distance_threshold;
        std::vector<cv::Scalar> text_colors;
        array<double,3> cov2ellipse(double Kxx, double Kxy, double Kyy);
        int intensity_range(const  cv::Mat &gray_img, int x, int y, float step_x, float step_y, int step_length);
        int intensity_range(const  cv::Mat &gray_img, int x, int y, int window_size);
        /// @brief 轮廓转换为shape 计算相关矩
        /// @param contour 
        /// @return 
        Shape contour2shape(const vector<cv::Point2i> &contour);
        /// @brief 计算kxx，kyy，kxy用于计算形状的不确定度
        /// @param contour 输入轮廓
        /// @param shape 输入形状
        /// @param gray_img 
        /// @param grad_x3 dx
        /// @param grad_y3 dy
        /// @return 
        bool cal_shape_cov(const vector<cv::Point2i> &contour, Shape* shape,  const  cv::Mat &gray_img, const  cv::Mat &grad_x3, const cv::Mat &grad_y3);
        /// @brief 检测圆 按照流程 进行阈值，轮廓查找 椭圆性测试，不确定度对比，整理
        /// @param input_img 输入图像
        /// @param output_img 输出debug
        /// @param target 输出shape
        /// @param debug 
        /// @return 成功与否
        bool detect_circles(cv::Mat& input_img, cv::Mat& output_img, vector<Shape>&target, bool debug=false);
        bool ellipse_test(const Shape &shape);
        /// @brief opencv类似方法对无序坐标进行排序，按照nx，ny形状整理
        /// @param source 无序坐标
        /// @param dist 整理后坐标
        void sortTarget(vector<cv::Point2f>&source, vector<cv::Point2f>&dist);
        static bool circle_compare(circle_info circle1,  circle_info circle2);

        void update_autocorrelation(cv::Mat &src, vector<Shape>& control_shapes);

        void visualize_result(cv::Mat& img_output, pair<bool,vector<Shape>> &result);

        //for exp
        bool detect_circles_banilar(cv::Mat& img, cv::Mat& img_output,vector<Shape>&target, bool debug);
        
};
#endif