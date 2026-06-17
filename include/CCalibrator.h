#ifndef __CCALIBRATOR_H__
#define __CCALIBRATOR_H__

#include <math.h>
#include <fstream>
#include <iostream>

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "CMomentsTracker.h"
#include "CLieAlgebra.h"
#include "utils.h"
#include "CFunctors.h"

#include <time.h>
#include <sys/time.h>
#include <unistd.h>

using namespace std;

using ceres::NumericDiffCostFunction;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;


class Calibrator{
    public:
<<<<<<< HEAD
        /// @brief 构造函数
        /// @param n_x 标定板宽
        /// @param n_y 标定板高
        /// @param n_d 相机畸变参数个数
        /// @param r 标定板圆半径
        /// @param distance 标定板圆距离
        /// @param max_scene 图像场景数
        /// @param result_path 结果保存目录
        Calibrator(int n_x, int n_y, int n_d,double r, double distance, int max_scene, string result_path);
=======
        Calibrator(int n_x, int n_y, bool asymmetric, int n_d,double r, double distance, int max_scene, string result_path);
>>>>>>> origin/main
        ~Calibrator() = default;

        void init();//清理外参列表
        /// @brief 输入target ，计算H矩阵Hs
        /// @param target 
        void inputTarget(vector<Shape> target);
        /// @brief zhang 方法，给定初始内参，由绝对二次曲线约束计算外参
        /// @param inital_params 
        /// @return 
        bool cal_initial_params(Params* inital_params);
        Params calibrate(int mode, bool save_jacob = false);
        
        void printParams(Params p);
        int get_num_scene();
        void save_extrinsic(string root_dir);
        void save_extrinsic();
        vector<se3> get_extrinsic();
        /// @brief 外参优化
        /// @param inital_params 初始参数
        /// @param mode 
        void update_Es(Params inital_params, int mode);

        Params calibrate_test(int mode,Params initial_params);
        
        void set_image_size(int _width, int _height);
        // for exp
        void visualize_rep(string path, Params params, int mode);
        // Params calibrate();

        // Eigen::Matrix3d get_extrinsic(Params params, int target_index);
        // Params batch_optimize(std::vector<int> sample, Params initial_params,int phase, double ratio);

    private:
        string root_dir;
        int height, width;
        int num_scene;
        int max_scene;
        int n_x, n_y, n_d;
        bool is_asymmetric;
        double distance;
        double original_r;
        // double curr_r;
        // vector<double> rs;
        double thr_homography;

        vector<se3> Es; // 外参列表 se3: rot , trans
        vector<pair<Eigen::Matrix3d,double>> Hs;//图像到标定板的单应矩阵

        vector<vector<Shape>> targets;
        vector<vector<Shape>> ud_targets;
        // vector<Eigen::Matrix3d> origin_conics;
        vector<Shape> origin_target;
        array<double, 4> ori_ms; //original_mean_sigma


        Params batch_optimize(std::vector<int> sample, Params initial_params, int mode, bool save_jacob = false, bool fix_intrinsic=false); //unc 반영
        double cal_reprojection_error(std::vector<int> sample,Params params, int mode);
        double cal_calibration_quality(std::vector<int> sample,Params params, int mode);
        void save_data_for_gpr(std::vector<int> sample,Params params, int mode);
        void set_origin_target();
        /// @brief 归一化外参，防止标定板位于相机后方
        void normalize_Es();

        //jang's 由张氏的内参获取初始的外参
        void set_inital_Es(Params params);
        array<double, 4> get_mean_sigma(const vector<Shape> &target);
        Eigen::VectorXd get_Vij(const Eigen::Matrix3d &H, int i, int j);
        /// @brief 获取每个图像的单应矩阵
        /// @param target 图像上的圆目标
        /// @return 单应矩阵 // unc 반영
        pair<Eigen::Matrix3d,double> get_H(const vector<Shape> &target); 
        ///绝对二次曲线约束 在标定过程中，标定板（通常是棋盘格）定义了一个世界坐标系平面（令 $Z=0$）。对于第 $i$ 张图像，标定板平面到图像平面之间的映射可以用一个 $3 \times 3$ 的单应性矩阵 $H_i = [h_1, h_2, h_3]$ 表示。相机内参矩阵为 $A$，定义一个对称矩阵 $B$（即代码中的 b）：
        Eigen::VectorXd get_absolute_conic();
        Eigen::MatrixXd normalize_matrix(Eigen::MatrixXd matrix);
};


#endif