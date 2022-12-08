#ifndef _DIFF_CHASSIS_HPP_
#define _DIFF_CHASSIS_HPP_

#include "base_chassis.hpp"

namespace motion{

/// @brief 差速底盘
class DiffChassis : public CarBase
{
private:
    double track_width_;        // 轮距，单位：m
    double speed_l_;            // 左轮轮面速度，单位：m/s
    double speed_r_;            // 右轮轮面速度，单位：m/s
    double wheel_perimeter_l_;  // 左轮周长长度，单位：m
    double wheel_perimeter_r_;  // 右轮周长长度，单位：m
    double reduction_ratio_l_;  // 左轮减速比
    double reduction_ratio_r_;  // 右轮减速比
    int motor_speed_l_;         // 左轮电机转速，单位：r/min
    int motor_speed_r_;         // 右轮电机转速，单位：r/min

    double v_;  // 目标线速度
    double w_;  // 目标角速度

    /// @brief 对差速底盘进行运动学分解，速度分发至每个驱动电机
    /// @param v 车辆中心的线速度
    /// @param w 车辆中心的角速度
    void kinematicDecomposition(double v, double w);

private:
    DiffChassis(double length, double width);
    static DiffChassis* pDiffSingleton;
    static bool is_instance_idle;

public:
    ~DiffChassis();

    void setTrackWidth(double tra_wid);
    void setPerimeter(double peri_l, double peri_r);
    void setGearRatio(double ratio_l, double ratio_r);
    void setCtrlAmount(double v, double w);
    void setCtrlInput(double v, double w);
    void getCtrlOutput(double& spd_l, double& spd_r);

    void getCurSpeed(double&v, double& w);

    /// @brief 获取单例模式实例
    /// @return 若成功，则返回实例指针，失败则返回空指针
    static DiffChassis* getInstance();

    /// @brief 释放单例模式实例
    /// @param instance 欲释放的实例指针
    /// @return 释放成功返回true，失败返回false
    static bool releaseInstance(void* instance);
};

typedef std::unique_ptr<DiffChassis> DiffChassis_ptr;

} // namespace motion

#endif