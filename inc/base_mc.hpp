#ifndef _BASE_MC_HPP_
#define _BASE_MC_HPP_

#include <sys/time.h>
#include <memory>
#include "math.h"
// #include <Eigen/Dense>
// using Eigen::MatrixXd;
// using Eigen::VectorXd;

namespace motion
{

#define T_INT 0.02      // 控制周期，单位：s
#define T_INT_US    (T_INT * 1000000)   // 控制周期，单位：us
// const double M_PI_2 = 1.57079632679489661923; /* pi/2 */
// const double M_PI_4 = 0.78539816339744830962; /* pi/4 */

const double MAX_V = 2.7778; // 手动控制最大运动线速度 = (10 / 3.6) m/s
const double MAX_W = M_PI_4; // 手动控制最大运动角速度 = PI/4 rad/s

const double MIN_V = 0.001; //车辆最小线速度
const double MIN_W = 0.001; //车辆最小角速度

// 底盘类型
typedef enum
{
    CHASSIS_DIFF,       // 差速底盘
    CHASSIS_ACKERMAN    // 阿克曼底盘
}CAR_CHASSIS_TYPE;

typedef enum
{
    SYS_UINIT,
    SYS_INIT,
    SYS_INIT_ERROR
}SYS_INIT_STAT;

typedef enum
{
    CAR_RUN,
    CAR_PAUSE,
    CAR_STOP
}CAR_STAT;

typedef struct
{
    double x;
    double y;
    double yaw;
}CAR_POSE;

//坐标姿态结构体
typedef struct
{
	float ssPosX;  //横坐标X（单位：mm）
	float ssPosY;  //竖坐标Y（单位：mm）
	float ssPosQ;  //航向角Q（单位：rad）
}ST_POT;

typedef struct
{
	float pos_x;      	// 坐标x
	float pos_y;	   	// 坐标y
	float pos_theta;  	// 坐标theta
}sr_pose_t;

typedef struct 
{
    double max_v;
    double end_v;
    double acc_up;
    double acc_down;
    double remain_dist;
} sr_velocity_plan_t;

// 获取系统时间，单位us
long getUsTime();
} // namespace motion

#endif