#ifndef _CAR_CTRL_HPP_
#define _CAR_CTRL_HPP_

#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "diff_chassis.hpp"
#include "ackerman_chassis.hpp"
#include "state_machine.hpp"
#include "target_tracking.hpp"
#include "line_path.hpp"
#include "base_algorithm.hpp"
#include "speed_planning.hpp"
#include "manual_ctrl.hpp"
#include "base_mc.hpp"
// #include "controlcan.h"
#include "VCU.h"

namespace motion{

class CarCtrl// : public CtrlBase
{
private:
    // 创建互斥锁
    static std::mutex mutex_motion_ctrl;
    static void threadMotorDriveCtrl(CarCtrl *self);

    // 单例模式成员变量
    static CarCtrl* pCarCtrlSingleton;
    static bool is_instance_idle;

    // 成员变量
    double v_auto_;
    double w_auto_;
    double v_manual_tar_;
    double w_manual_tar_;
    CAR_POSE target_pos_;

    // 创建车辆实例对象指针
    VCU *pVcuCtrl;
    CarBase* pCar;
    ManualCtrl *pManual;
    TargetTracking *pTracking;

    // 指令记录指针
    MC_MCMD cmd_ready;
    bool cmd_decelerating;

    MC_MODE motion_ctrl_mode;
    MC_STAT motion_ctrl_stat;
    MC_MCMD motion_ctrl_mcmd;
    CAR_CHASSIS_TYPE chassis_;
    SYS_INIT_STAT sys_init_stat_;
    
    int sendCmd(MC_MCMD cmd);
    void process_idle(double& v_cmd, double& w_cmd);
    void process_manual(double& v_cmd, double& w_cmd);
    void process_auto(double& v_cmd, double& w_cmd);
    void process_track(double& v_cmd, double& w_cmd);

    int motionMainLogicCtrl();
    void interpolationCtrl();
    void stateMachineUpdate();
    bool releaseCarInstance(CarBase* p);
    CarBase* getCarInstance(CAR_CHASSIS_TYPE chassis);

private:
    CarCtrl();

public:
    ~CarCtrl();

    /// 车辆控制函数
    int start();
    int hold();
    int stop();

    /// @brief 初始化运动控制模块
    /// @param chassis 底盘类型
    /// @return 返回0成功，其余失败
    int initMotionCtrlSystem(CAR_CHASSIS_TYPE chassis);

    /// @brief 获取当前车辆状态
    /// @param mode 车辆当前的运行模式
    /// @param stat 车辆当前的运行状态
    /// @return 返回0成功，其余失败
    int getCarStatus(MC_MODE& mode, MC_STAT& stat);

    /// @brief 获取车辆当前线速度和角速度
    /// @param v 线速度
    /// @param w 角速度
    /// @return 返回零表示成功，其余失败
    int getCurSpeed(double& v, double& w);

    /// @brief 紧急停止
    /// @return 
    int emergencyStop();

    /// @brief 急停退出
    /// @return 
    int emergencyExit();

    /// @brief 切换底盘控制模式
    /// @param mode 需要切换到的模式
    /// @return 返回零代表模式切换成功
    int switchMode(MC_MODE mode);

    /// @brief 获取底盘类型
    /// @return 
    CAR_CHASSIS_TYPE getCarChassis();

    /// @brief 获取系统初始化状态
    /// @return 
    SYS_INIT_STAT getSysStatus();

    /// @brief 设置自由导航模式下的车辆线速度和角速度
    /// @param v 线速度值
    /// @param w 角速度值
    void setAutoSpeed(double v, double w);

    /// @brief 设置手动控制目标线速度和角速度
    /// @param v 线速度值
    /// @param w 角速度值
    void setManualSpeed(double v, double w);

    /// @brief 设置跟踪目标位置
    /// @param tar_pose 
    void setTrackPos(CAR_POSE tar_pos);

    /// @brief 更新车辆当前位置
    /// @param cur_pos 
    void updatePose(CAR_POSE cur_pos);

    /// @brief 获取单例模式实例
    /// @return 若成功，则返回实例指针，失败则返回空指针
    static CarCtrl* getInstance();

    /// @brief 释放单例模式实例
    /// @param instance 欲释放的实例指针
    /// @return 释放成功返回true，失败返回false
    static bool releaseInstance(void* instance);
};


} // namespace motion


#endif