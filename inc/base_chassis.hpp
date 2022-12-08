#ifndef _BASE_CHASSIS_HPP_
#define _BASE_CHASSIS_HPP_

#include "./base_mc.hpp"

namespace motion{
    
// 车辆基类参数
class CarBase
{
private:
    double car_length_;          // 车长，单位m
    double car_width_;           // 车宽，单位m
    double car_wheel_base_;      // 轴距，单位m
    double car_track_width_;     // 轮距，单位m
    double car_reduction_ratio_; // 减速比
    double car_wheel_diameter_;  // 轮径，单位m

    double car_v_;      // 车体中心线速度，单位m/s;
    double car_w_;      // 车体中心角速度，单位rad/s;

    double max_vel_;    // 车辆的最大速度
    double max_acc_;    // 车辆的最大加速度
    
    double acc_v_;      // 自动控制线加速度
    double dec_v_;      // 自动控制线减速度
    double acc_w_;      // 自动控制角加速度
    double dec_w_;      // 自动控制角减速度

    double manual_acc_v_;   // 手动控制线加速度
    double manual_dec_v_;   // 手动控制线减速度
    double manual_acc_w_;   // 手动控制角加速度
    double manual_dec_w_;   // 手动控制角减速度

    // Vector3d car_pose_;  // x y z 车辆当前姿态
    // Vector3d car_pose_last_;  // x y z

    CAR_POSE cur_pose_;         // 当前位姿
    CAR_POSE tar_pose_;         // 目标位姿
    CAR_CHASSIS_TYPE car_type_; // 车辆类型

public:
    CarBase(CAR_CHASSIS_TYPE type = CHASSIS_DIFF)
        : car_type_(type){}

    virtual ~CarBase(){}

    const CAR_CHASSIS_TYPE &getType() const {
        return car_type_;
    }

    // 获取车辆长度
    double getLength() const {
        return car_length_;
    }

    // 设置车辆长度
    void setLength(double length){
        car_length_ = length;
    }

    // 获取车辆宽度
    double getWidth() const {
        return car_width_;
    }

    // 设置车辆宽度
    void setWidth(double width) {
        car_width_ = width;
    }

    // 设置电机速度
    void setMotorSpeed(int no, int speed){

    }

    // 更新位姿
    void updatePose(CAR_POSE cur_pose){
        cur_pose_ = cur_pose;
    }

    // 正向运动学分解
    virtual void kinematicDecomposition(double v, double w) = 0;
    virtual void getCtrlOutput(double& ctrl1, double& ctrl2) = 0;
    virtual void setCtrlInput(double v, double w) = 0;
    virtual void getCurSpeed(double&v, double& w) = 0;
    // virtual void getCurStatus(CAR_STAT& stat) = 0;
};

typedef std::unique_ptr<CarBase> CarBase_ptr;

} // namespace motion

#endif