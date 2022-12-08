#ifndef _SPEED_PLANNING_HPP_
#define _SPEED_PLANNING_HPP_

#include "math.h"

namespace motion{
    
typedef struct 
{
    double max_v;
    double min_v;
    double end_v;
    double acc;
    double dec; //此处需为正值
    double rem_dis;
}vel_plan_t;

class SpeedPlanning
{
private:
    /* data */
public:
    SpeedPlanning(/* args */){};
    ~SpeedPlanning(){};

    void T_shaped_speed_planning(vel_plan_t restrict, float dt, float& cur_v){
        bool dec_flag = false;
        double dis_need = fabs(pow(restrict.end_v, 2) - pow(cur_v, 2)) / (2 * restrict.dec);
        if(restrict.rem_dis < dis_need) dec_flag = true;

        if(dec_flag){
            double dec_dyn = fabs(pow(restrict.end_v, 2) - pow(cur_v, 2)) / (2 * restrict.rem_dis);
            dec_dyn = (dec_dyn > (restrict.dec + 0.5)) ? (restrict.dec + 0.5) : dec_dyn;
        }else{
            cur_v = ((cur_v + restrict.acc * dt) > restrict.max_v) ? restrict.max_v : (cur_v + restrict.acc * dt);
        }

        // 
        cur_v = (cur_v > restrict.min_v) ? cur_v : restrict.min_v;
    }
};



// void sr_velocity_control(sr_velocity_plan_t velocity, float dt, float* current_v){
//     unsigned char acc_down_flag;
//     uint8_t AccDownCorrectFlag;
//     float AccDown;
//     float dist_offset = 0.005f;

//     if((velocity.remain_dist - 0.01f) < (pow(*current_v, 2) - pow(velocity.end_v, 2)) / 2.0f / velocity.acc_down){
//         acc_down_flag = 1;
//     }
//     else{
//         acc_down_flag = 0;
//     }

//     if(acc_down_flag == 0){
//         if(fabs(*current_v) <velocity.max_v){
//             *current_v = fabs(*current_v) + velocity.acc_up * dt;
//         }
//         else{
//             *current_v = fabs(*current_v) - velocity.acc_down * dt;
//         }
//     }
//     if(acc_down_flag == 1){
//         if(velocity.remain_dist > 0.01f){
//             AccDown = (pow(*current_v, 2) - pow(velocity.end_v, 2)) / 2.0f / fabs(velocity.remain_dist  - fabs(*current_v) * 0.02f - dist_offset);
//             // AccDown = ClipFloat(AccDown, -1, 1); //速度限幅
//             if(AccDown > fabs(velocity.acc_down) + 0.5f){
//                 AccDown = velocity.acc_down + 0.5f;
//             }
//             else if(AccDown < -1.0f * fabs(velocity.acc_down) - 0.5f){
//                 AccDown = -1.0f * fabs(velocity.acc_down) - 0.5f;
//             }
//             AccDownCorrectFlag = 1;
//         }
//         else{
//             AccDownCorrectFlag = 0;
//         }

//         if (AccDownCorrectFlag == 1){
//             *current_v = fabs(*current_v) - AccDown * dt;	
//         }
//         else{
//             *current_v = fabs(*current_v) - velocity.acc_down * dt;	
//         }
                    
//     }
//     if(*current_v < dist_offset){
//         *current_v = dist_offset;
//     }
// }

} // namespace motion

#endif