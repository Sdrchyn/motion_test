#ifndef _BASE_ALGORITHM_HPP_
#define _BASE_ALGORITHM_HPP_

#include "math.h"
#include "base_mc.hpp"

namespace motion{
    
//向量有关结构体
typedef struct
{
	float ssVx;      	//X方向差
	float ssVy;	   		//Y方向差
	float fpLength; 	//向量长度（单位mm）
	float fpalpha;  	//向量与Y轴角度（单位:弧度）
}ST_VECTOR;

class AlgorithmBase
{
private:
    /* data */
public:
    AlgorithmBase(/* args */){};
    ~AlgorithmBase(){};

    /**
     * @brief 根据目标点计算角速度参数
     * 
     * @param current_position 
     * @param direction 
     * @param target_position 
     * @return float 
     */
    float sr_calc_coeff(sr_pose_t current_position, unsigned char direction, sr_pose_t target_position){
        float xita_V = 0.0f;
        sr_pose_t TransTargetPosition;
        
        if(direction == 0x01){
            xita_V = current_position.pos_theta;
        }
        else if(direction == 0x02){
            xita_V = current_position.pos_theta + 3.14159f;
        }

        // 计算目标点向量
        double dx = target_position.pos_x - current_position.pos_x;
        double dy = target_position.pos_y - current_position.pos_y;

        //计算导航圆弧的曲率
        //将target_position转换为机器人坐标系的坐标
        TransTargetPosition.pos_x = (target_position.pos_x - current_position.pos_x) * cosf(xita_V) + (target_position.pos_y - current_position.pos_y) * sinf(xita_V);
        TransTargetPosition.pos_y = -(target_position.pos_x - current_position.pos_x) * sinf(xita_V) + (target_position.pos_y - current_position.pos_y) * cosf(xita_V);
        //求出D的平方
        float SquareD = TransTargetPosition.pos_x * TransTargetPosition.pos_x + TransTargetPosition.pos_y * TransTargetPosition.pos_y;
        //将绝对坐标系下的圆心点转化为车体坐标系下的点。只需要求y即可
        //求圆弧的曲率，此处用相同大小角度正弦值形成一个等式来求解
        float gama = 2.0f * TransTargetPosition.pos_y / SquareD;
        return gama;
    }

    /// @brief 计算向量点乘
    float sr_dot_product(float x1, float y1, float x2, float y2){
        return (x1 * x2 + y1 * y2);
    }

    /// @brief 计算两点之间最短直线
    float sr_dist(sr_pose_t a, sr_pose_t b){
        float diff_x = a.pos_x - b.pos_x;
        float diff_y = a.pos_y - b.pos_y;
        return sqrtf(diff_x * diff_x + diff_y * diff_y);
    }

    /// 2@brief 计算点到直线的最短距离
    float sr_nearest_point_to_line(sr_pose_t start, sr_pose_t end, sr_pose_t current_point, sr_pose_t* pb){
        float vx,vy, wx, wy;
        float c1, c2, di, b;

        vx = end.pos_x - start.pos_x;
        vy = end.pos_y - start.pos_y;

        wx = current_point.pos_x- start.pos_x;
        wy = current_point.pos_y - start.pos_y;

        c1 = sr_dot_product(wx, wy, vx, vy);
        c2 = sr_dot_product(vx, vy, vx, vy);

        if(c2 <= c1){
            di = sr_dist(current_point, end);
            pb->pos_x = end.pos_x;
            pb->pos_y = end.pos_y;
            return di;
        }

        b = c1 / c2;
        pb->pos_x = start.pos_x + b * vx;
        pb->pos_y = start.pos_y + b * vy;

        di = sr_dist(current_point, *pb);

        return di;
    }

    /**
     * @brief  计算一向量在基准向量方向投影
     * @param  stAim: 目标向量，即需要投影的向量
     *         stBase:基准向量
     * @retval 目标向量在基准向量上的投影的距离。
     * @note 当目标点在基准直线之间时，投影值为正，反之为负
     */
    float CalRadialProjection(ST_VECTOR stAim, ST_VECTOR stBase){
        return (float)(stBase.ssVx * stAim.ssVx + stBase.ssVy * stAim.ssVy) / sqrtf((float)(stBase.ssVy * stBase.ssVy + stBase.ssVx * stBase.ssVx)) ;		
    }

    /**
     * @brief 互补滤波
     * 
     * @param porigin_pose 
     */
    // void position_comlementary_filter(sr_ev_pose_t *porigin_pose)
    // {
    //     static uint32_t timestamp;
    //     static uint32_t time_pirnt;
        
    //     pose_fusion.x = porigin_pose->x / 10000.0f;
    //     pose_fusion.y = porigin_pose->y / 10000.0f;
    //     pose_fusion.yaw = porigin_pose->yaw / 10000.0f;

    //     float dt = get_delta_time( timestamp, get_system_tick()) / 1000.0f;
    //     timestamp = get_system_tick();

    //     float k = 0.9;//融合系数

    //     float vx = sr_get_vehicle_v();
    //     float vy = 0.0;
    //     float w = sr_get_vehicle_w();

    //     float x_ = pose_fusion.fusion_x + vx*dt*cosf(pose_fusion.fusion_yaw) - vy * dt * sinf(pose_fusion.fusion_yaw);
    //     float y_ = pose_fusion.fusion_y + vx*dt*sinf(pose_fusion.fusion_yaw) + vy * dt * cosf(pose_fusion.fusion_yaw);
    //     float yaw_ = pose_fusion.fusion_yaw + w * dt;

    //     if((yaw_ < 0) || (pose_fusion.yaw < 0)){
    //         //防止在 -3.14 和3.14处出问题 小于零的时候加一圈
    //         float dyaw = sr_convert_angle(yaw_ - pose_fusion.yaw);
    //         yaw_ += 6.28318;
    //         pose_fusion.yaw = yaw_ - dyaw;
    //     }

    //     x_ = x_ * k + pose_fusion.x * (1-k);
    //     y_ = y_ * k  + pose_fusion.y * (1-k);
    //     yaw_ = yaw_ * k  + pose_fusion.yaw * (1-k);

    //     float err = (x_ - pose_fusion.x)*(x_ - pose_fusion.x) + (y_ - pose_fusion.y)*(y_ - pose_fusion.y);

    //     if(err > 0.0025){
    //         //滤波和定位误差不能大于5cm
    //         x_ = pose_fusion.x;
    //         y_ = pose_fusion.y;
    //     }
    //     if(fabs(yaw_ - pose_fusion.yaw) > 0.08726){
    //         //滤波和定位误差不能大于5°
    //         yaw_ = pose_fusion.yaw;
    //     }

    //     pose_fusion.fusion_x = x_;
    //     pose_fusion.fusion_y = y_;
    //     pose_fusion.fusion_yaw = sr_convert_angle(yaw_);

    //     stRobot.stPot.ssPosX = pose_fusion.fusion_x;
    //     stRobot.stPot.ssPosY = pose_fusion.fusion_y;
    //     stRobot.stPot.ssPosQ = sr_convert_angle(pose_fusion.fusion_yaw);

    //     if(get_delta_time( time_pirnt, get_system_tick()) > 300){
    //         time_pirnt = get_system_tick();
    //         if(err > 10.0f){
    //             sr_log(LOG_ERROR, "mc_pose delta distance has problem! distance is %f\r\n", err);
    //         }
    // }
};

} // namespace motion


#endif