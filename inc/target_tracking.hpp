#ifndef _TARGET_TRACKING_HPP_
#define _TARGET_TRACKING_HPP_

#include "math.h"
#include "./base_mc.hpp"

namespace motion{

class TargetTracking
{
private:
    double lookahead_dis_;
    double yaw_;
    double safe_dis_;
    CAR_POSE cur_pose_;
    CAR_POSE tar_pose_;

    /// @brief 计算导航圆弧的曲率
    /// @param cur 
    /// @param tar 
    /// @return 旋转半径对应的曲率，也即半径的倒数
    double calc_curvature(CAR_POSE cur, CAR_POSE tar);

public:
    TargetTracking();
    ~TargetTracking();

    // 设置跟踪目标安全距离
    void setSafeDis(double dis);

    // 设置前视距离
    void setLookaheadDis(double dis);

    // 更新车辆当前位置
    void updateCurPos(CAR_POSE cur_pose);

    // 更新目标点位置
    void updateTargetPos(CAR_POSE tar_pose);

    // 获取跟踪控制输出量
    void getCtrl(CAR_POSE cur_pose, double v, double& w);

    // 获取两个位置点的距离
    double getDistance(CAR_POSE pos1, CAR_POSE pos2);
};

}

#endif