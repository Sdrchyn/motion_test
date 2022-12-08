#pragma once
#include <boost/timer.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/noncopyable.hpp>
#include "VCUMonitor.h"


//档位定义
const unsigned char GEAR_PARK = 1;      //驻车挡
const unsigned char GEAR_BACKWARD = 2;  //后退挡
const unsigned char GEAR_FREE = 3;      //空挡
const unsigned char GEAR_FORWARD = 4;   //前进挡

const unsigned char PATK_RELEASE = 0;
const unsigned char PATK_APPLY = 1;

//control range
const float MAX_STEER = 30.0f;			//最大转向角
const float MAX_VELOCITY = 3.5f;		//最大速度(m/s)
const float MAX_BRAKEEFFORT = 100.0f;    //最大制动力(%)

//controlcan configure param




struct VCUMonitor;
struct VCUPrivate;
class VCU : boost::noncopyable
{
public:
    VCU();
    ~VCU();

    //启动
    int start();
    //停止
    int stop();

    void setEnable(bool enable);
    void setParking(bool on);
    void setOdometer(bool clr);
	void setStandby(bool standby);	//进入或退出待机状态
	void setEmergency();	//进入紧急状态
	void clearEmergency();	//退出紧急状态
    bool inEmergency();     //是否处于紧急状态
    void setWrenchEffort(float steerEffort/*-90 ~ 90*/, float propulsiveEffort/*0 ~ 10.24*/, float resistiveEffort/*0 ~ 100*/);

    void setMonitor(VCUMonitor* monitor);

private:
    VCUPrivate* _p;
};
