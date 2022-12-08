#ifndef _ACKERMAN_CHASSIS_HPP_
#define _ACKERMAN_CHASSIS_HPP_

#include "math.h"
#include "base_chassis.hpp"

namespace motion{

/// @brief 阿克曼底盘
class AckermanChassis : public CarBase
{
private:
    double track_width_;    // 轮距，单位：m
    double wheel_base_;     // 轴距，单位：m
    double speed_;          // 后驱轮面速度，单位：m/s
    double wheel_perimeter_;// 后驱外径长度，单位：m
    double reduction_ratio_;// 后驱减速比 gear_ratio
    double delta_angle_;    // 前轮转角，单位：rad
    double min_rotate_r_;   // 最小转弯半径，单位：m
    int motor_speed_;       // 后驱电机转速，单位：r/min

    double v_;  // 目标线速度
    double w_;  // 目标角速度

    /// @brief 对阿克曼底盘进行运动学分解，速度分发至每个驱动电机
    /// @param v 车辆中心的线速度
    /// @param w 车辆中心的角速度
    void kinematicDecomposition(double v, double w);

private:
    AckermanChassis(double length, double width);
    static AckermanChassis* pAckermanSingleton;
    static bool is_instance_idle;

public:
    ~AckermanChassis();

    void setTrackWidth(double tra_wid);
    void setWheelBase(double wheel_base);
    void setPerimeter(double peri);
    void setGearRatio(double ratio);
    void setCtrlInput(double v, double w);
    void getCtrlOutput(double& speed, double& delta);

    void getCurSpeed(double&v, double& w);

    /// @brief 获取单例模式实例
    /// @return 若成功，则返回实例指针，失败则返回空指针
    static AckermanChassis* getInstance();

    /// @brief 释放单例模式实例
    /// @param instance 欲释放的实例指针
    /// @return 释放成功返回true，失败返回false
    static bool releaseInstance(void* instance);
};

typedef std::unique_ptr<AckermanChassis> AckermanChassis_ptr;

} // namespace motion

#endif