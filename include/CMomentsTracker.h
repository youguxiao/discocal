#ifndef __CMOMENTSTRACKER_H__
#define __CMOMENTSTRACKER_H__

#include <vector>
#include <array>
#include <iostream>
#include "math.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "utils.h"
using namespace std;


class MomentsTrackerError: public std::exception{
    public:
        const char* what(){
            return "Value exceed the range";
        }
};

class MomentsTracker
{
private:
    /* data */
    int max_dim, max_n, comb_max;
    vector<vector<int>> comb_buffer;
    vector<vector<double>> moment_buffer;
    void init();//清理列表
    double _M_2i2j(int i, int j);
    /// @brief 对应论文公式 (18) 和 (19) 的权重系数 $w_{0r}$ 和 $w_{1r}$。径向畸变多项式系数 $\mathbf{d}$ 在雅可比行列式（面积微元变换）展开后，会组合成固定规律的系数阵，这两个函数负责组合这些系数。  
    /// @param n 迭代次数
    /// @param ds 畸变系数
    /// @return 权重系数
    double C0n(int n, vector<double> ds);
    double C1n(int n, vector<double> ds);
    double nCr(int i, int j);
    double I_2i2j(int i, int j);
    double M_2i2j(int i, int j);
    double M_2i2j(int i, int j, double a, double b);
    /// @brief 实现了论文中的 Theorem 2（旋转同变性） 和 Theorem 5（平移通用解）。它在 $O(1)$ 时间内，利用多项式展开，计算出任意摆放的椭圆在径向畸变微元 $s_n^r = (x^2+y^2)^r$ 作用下的各阶矩积分项 s0 (面积项), sx, sy 。  
    /// @param n 迭代次数
    /// @param a 椭圆长轴
    /// @param b 椭圆短轴
    /// @param tx 椭圆Tx 坐标
    /// @param ty 椭圆Ty 坐标
    /// @param alpha 椭圆角度
    /// @return 矩积分项
    array<double ,3> intSn(int n,double a, double b, double tx, double ty,double alpha);
    

public:
    MomentsTracker(int dim);
    MomentsTracker();
    ~MomentsTracker();

    /// @brief 最终的融合步。通过有限项（由畸变阶数决定，通常 max_dim=4）的线性求和，直接吐出由于畸变导致的偏心质心，完美避开了连续二维积分。
    /// @param Q 将归一化平面上的椭圆矩阵 $Q$（满足 $x^\top Q x = 0$）
    /// @param ds 畸变系数
    /// @return 畸变后的偏心质心
    Point ne2dp(Eigen::Matrix3d Q, vector<double> ds);
    Point wc2dp_Numerical(Eigen::Matrix3d Cw,Eigen::Matrix3d H, vector<double> ds,int total_iter=1600);
    Point ne2dp_Numerical(Eigen::Matrix3d Q, vector<double> ds);
    /// @brief 投影世界坐标到图像坐标
    /// @param wx 世界坐标x
    /// @param wy 世界坐标y
    /// @param r 圆半径
    /// @param intrinsic 内参 
    /// @param E 归一化平面单应映射矩阵
    /// @param mode 模式0 使用椭圆参数计算投影后中心，模式1 使用椭圆中心投影坐标 模式2 使用wx,wy投影坐标 模式3 使用数值解计算椭圆参数投影后中心
    /// @return 投影后图像坐标

     // 0: moments, 1: conic, 2: points, 3: ne2dp_Numerical, 4: wc2dp_Numerical
    Point project(double wx, double wy, double r,Params intrinsic, Eigen::Matrix3d E, int mode, bool toImage = true);
    array<double, 5> ellipse2array(Eigen::Matrix3d Q);
    Point distort_Point(Point pn, vector<double> ds);
};

















#endif