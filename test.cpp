#include <iostream>
#include <memory>
#include <glog/logging.h>
#include "car_ctrl.hpp"

int main(int argc, char* argv[])
{
    // FLAGS_logtostderr = to_stderr;
    // FLAGS_minloglevel = 0;
    // FLAGS_log_dir = "./log/";
    // google::InitGoogleLogging(argv[0]);
    // google::ShutdownGoogleLogging();

    motion::CarCtrl *pCarCtrl = motion::CarCtrl::getInstance();
    pCarCtrl->initMotionCtrlSystem(motion::CHASSIS_DIFF, "/dev/pts/3");
    pCarCtrl->switchMode(motion::MODE_MANUAL);
    pCarCtrl->start();
    long start_time = motion::getUsTime();
    long first_time = start_time;
    long time_enter = start_time;
    double v_set = 2.0;
    double w_set = 1.0;
    pCarCtrl->setManualSpeed(v_set, w_set);

    motion::MC_MODE mode;
    motion::MC_STAT stat;

    // int rtn = switchMode(MODE_AUTO);
    // sendCmd(MCMD_RUN);
    bool flag = false;
    bool flag1 = false;
    while(1)
    {
        long cur_time = motion::getUsTime();
        
        if (cur_time - start_time > 20000)
        {
            pCarCtrl->getCarStatus(mode, stat);

            switch (mode)
            {
                case motion::MODE_MANUAL:
                {
                    double v, w;
                    pCarCtrl->getCurSpeed(v, w);
                    if(v >= 1.999 && w >= 0.999)
                    {
                        pCarCtrl->stop();
                    }


                    if(stat == motion::STAT_STOP)   pCarCtrl->switchMode(motion::MODE_AUTO);
                }break;

                case motion::MODE_AUTO:
                {
                    
                    if((stat != motion::STAT_RUN) && (!flag))
                    {
                        flag = true;
                        pCarCtrl->start(); time_enter = cur_time;
                        LOG(INFO) << "***************************************************************************";
                    }
                    double v = sin(M_PI / 200 * ((cur_time - first_time) / 20000));
                    double w = cos(M_PI / 200 * ((cur_time - first_time) / 20000));
                    

                    if((cur_time - time_enter) < 5000000)
                    {
                        pCarCtrl->setAutoSpeed(v, w);
                    }
                    else
                    {
                        pCarCtrl->stop();

                        if(stat == motion::STAT_STOP)
                        {
                            pCarCtrl->switchMode(motion::MODE_TRACK);
                        }
                    }  
                }break;

                case motion::MODE_TRACK:
                {
                    
                    if((stat != motion::STAT_RUN) && (!flag1))
                    {
                        flag1 = true;
                        pCarCtrl->start(); time_enter = cur_time;
                        LOG(INFO) << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
                    }

                    motion::CAR_POSE pos;
                    pos.x = 5 + 5 * sin(M_PI / 200 * ((cur_time - first_time) / 20000));
                    pos.y = 5 + 5 * cos(M_PI / 200 * ((cur_time - first_time) / 20000));
                    
                    LOG(INFO) << "X&Y: " << pos.x << " " << pos.y;

                    if((cur_time - time_enter) < 5000000)
                    {
                        pCarCtrl->setTrackPos(pos);
                    }
                    else
                    {
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