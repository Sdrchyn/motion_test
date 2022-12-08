#ifndef _MANUAL_CTRL_HPP_
#define _MANUAL_CTRL_HPP_

#include "./base_mc.hpp"
#include "math.h"

// const double MAX_V = 2.5;   // 手动控制最大运动线速度 V = 2.5 m/s
// const double MAX_W = M_PI_4;// 手动控制最大运动角速度 W = π/4 rad/s

namespace motion{
    
class ManualCtrl
{
private:
    double v_;  // 目标线速度
    double w_;  // 目标角速度

    double v_cur_;  // 手动控制当前线速度
    double w_cur_;  // 手动控制当前角速度

    double v_acc_;  // 手动控制线加速度
    double v_dec_;  // 手动控制线减速度
    double w_acc_;  // 手动控制角加速度
    double w_dec_;  // 手动控制角减速度

    int sign(int val);
    double deltaLimit(double val, double min, double max);

public:
    ManualCtrl();
    ~ManualCtrl();

    // 设置手动控制线速度和角速度，角速度随速度值成正弦变化
    void setManualSpeed(double v, double w);

    // 设置手动控制线加减速度和角加减速度
    void setManualAccDec(double av, double dv, double aw, double dw);

    // 获取手动控制目标线速度和目标角速度
    void getTargetSpeed(double& v, double& w);

    // 获取手动控制当前线速度和角速度
    void getCurSpeed(double& v, double& w);

    // 手动控制主程序
    void manualControl();
};

typedef std::shared_ptr<ManualCtrl> ManualCtrl_ptr;

} // namespace motion


#endif