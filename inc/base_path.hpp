#ifndef _BASE_PATH_HPP_
#define _BASE_PATH_HPP_

namespace motion{

enum PATH_TYPE{
    PATH_LINE,  //直线路径
    PATH_ROTATE,//旋转路径
    PATH_BEZIER,//贝塞尔路径
    PATH_SPLINE //样条曲线路径
};

class PathBase
{
private:
    double xs_;
    double ys_;
    double zs_;
    double xe_;
    double ye_;
    double ze_;

    //路径起始终点速度限制
    double v_s_;
    double v_e_;

    //路径允许最大最小速度
    double v_max_;
    double v_min_;

    //路径允许最大最小加速度
    double a_max_;
    double a_min_;

    //路径类型
    PATH_TYPE type_;

public:
    PathBase(PATH_TYPE type = PATH_LINE)
        : type_(type){};
    ~PathBase(){};

    /// @brief 计算前视距离
    /// @param v 当前速度
    /// @param min_distance 最小前视距离 
    /// @return 
    double calc_lookahead_dist(double v, double min_distance)
    {
        float lookahead_dist = 0.8f * (fabs(v) + 0.1f);
        // 1.7f * (fabs(pstNav->stVeltDes.fpV)) -0.2;

        lookahead_dist = (lookahead_dist >= min_distance) ? lookahead_dist : min_distance;

        return lookahead_dist;
    }
};

} // namespace motion


#endif