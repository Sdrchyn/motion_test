#include <iostream>
#include <memory>
#include <glog/logging.h>
#include "car_ctrl.hpp"
#include "serial_comm.hpp"

int main(int argc, char* argv[])
{
    // FLAGS_logtostderr = to_stderr;
    // FLAGS_minloglevel = 0;
    // FLAGS_log_dir = "./log/";
    // google::InitGoogleLogging(argv[0]);
    // google::ShutdownGoogleLogging();

    //创建控制对象并实例化
    motion::CarCtrl *pCarCtrl = motion::CarCtrl::getInstance();

    //初始化控制系统
    pCarCtrl->initMotionCtrlSystem(motion::CHASSIS_DIFF, "/dev/pts/3");

    //切换到手动控制模式
    pCarCtrl->switchMode(motion::MODE_MANUAL);

    //启动手动控制
    pCarCtrl->start();

    //获取系统时间，单位：us
    long start_time = motion::getUsTime();
    long first_time = start_time;
    long time_enter = start_time;

    //设置手动控制目标速度
    double v_set = 2.0;
    double w_set = 1.0;
    pCarCtrl->setManualSpeed(v_set, w_set);

    //定义模式和状态变量，供车辆状态读取函数使用
    motion::MC_MODE mode;
    motion::MC_STAT stat;

    // int rtn = switchMode(MODE_AUTO);
    // sendCmd(MCMD_RUN);
    bool flag = false;
    bool flag1 = false;

    // const char device_serial[20] = "/dev/pts/3";
    // serial_init(device_serial);
    
    while(1)
    {
        long cur_time = motion::getUsTime();
        

        //如果前后两次调用周期大于20ms，则再次进入
        if (cur_time - start_time > 20000)
        {
            //send_speed(2000, 5000);

            //获取车辆当前状态
            pCarCtrl->getCarStatus(mode, stat);

            //根据当前模式进行对应的处理
            switch (mode)
            {

                //手动控制模式
                case motion::MODE_MANUAL:
                {
                    //获取车辆当前速度
                    double v, w;
                    pCarCtrl->getCurSpeed(v, w);
                    

                    if(v >= 1.999 && w >= 0.999)
                    {
                        //发送停止指令
                        pCarCtrl->stop();
                    }

                    //如果车辆已经处于停止状态，切换车辆模式到自动导航模式
                    if(stat == motion::STAT_STOP)   pCarCtrl->switchMode(motion::MODE_AUTO);
                }break;

                // 自动控制模式
                case motion::MODE_AUTO:
                {
                    // 如果自动模式下尚未启动
                    if((stat != motion::STAT_RUN) && (!flag))
                    {
                        flag = true;
                        pCarCtrl->start(); time_enter = cur_time;
                        LOG(INFO) << "***************************************************************************";
                    }
                    double v = sin(M_PI / 200 * ((cur_time - first_time) / 20000));
                    double w = cos(M_PI / 200 * ((cur_time - first_time) / 20000));
                    
                    //启动后的5秒内都进行自动控制，运行超出5秒后直接停止
                    if((cur_time - time_enter) < 5000000)
                    {
                        //发送自动控制的线速度和角速度
                        pCarCtrl->setAutoSpeed(v, w);
                    }
                    else
                    {
                        //发送停止指令
                        pCarCtrl->stop();

                        //如果车辆已经处于停止状态，切换到跟踪模式
                        if(stat == motion::STAT_STOP)
                        {
                            pCarCtrl->switchMode(motion::MODE_TRACK);
                        }
                    }  
                }break;

                //跟踪模式
                case motion::MODE_TRACK:
                {
                    //如果尚未启动，则启动
                    if((stat != motion::STAT_RUN) && (!flag1))
                    {
                        flag1 = true;
                        pCarCtrl->start(); time_enter = cur_time;
                        LOG(INFO) << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
                    }

                    //随机生成跟踪位置点
                    motion::CAR_POSE pos;
                    pos.x = 5 + 5 * sin(M_PI / 200 * ((cur_time - first_time) / 20000));
                    pos.y = 5 + 5 * cos(M_PI / 200 * ((cur_time - first_time) / 20000));

                    if((cur_time - time_enter) < 5000000)
                    {
                        //发送目标位置进行跟踪
                        pCarCtrl->setTrackPos(pos);
                    }
                    else
                    {
                        //停止并切换模式
                        pCarCtrl->stop();

                        if(stat == motion::STAT_STOP)
                        {
                            pCarCtrl->switchMode(motion::MODE_IDLE);
                        }
                    }  
                }break;
                
                default:
                    break;
            }

            start_time = cur_time;
        }        
    }

    return 0;
}