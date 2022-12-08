#ifndef _LINE_PATH_HPP_
#define _LINE_PATH_HPP_

#include "math.h"
#include "base_path.hpp"

namespace motion{
    
class PathLine : public PathBase
{
private:
    //路径起始和终点坐标值
    double x_s_;
    double y_s_;
    double x_e_;
    double y_e_;

public:
    PathLine() : PathBase(PATH_LINE){};
    ~PathLine(){};

    double getLineLength() const{
        double dx = x_e_ - x_s_;
        double dy = y_e_ - y_s_;
        return sqrt(pow(dx, 2) + pow(dy, 2));
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

    /// @brief 计算点到直线的最短距离
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

    ///@brief 根据起点 终点 当前点 前世距离 计算出直线上的目标点
    sr_pose_t sr_calc_line_target_point(sr_pose_t start, sr_pose_t end, sr_pose_t current_point, float lookahead) {
        sr_pose_t stVecSE;
        sr_pose_t Pb1;
        sr_pose_t target_position;
        stVecSE.pos_x = end.pos_x - start.pos_x;
        stVecSE.pos_y = end.pos_y - start.pos_y;

        sr_nearest_point_to_line(start, end, current_point, &Pb1);
        float d1 = sr_dist(Pb1, end);
        float d3 = sr_dist(start, end);
        float d2 = d1 - lookahead;
        float t = (d3 - d2) / d3;
        if(t <= 0){
            t = 0;
        }
        target_position.pos_x = start.pos_x + (stVecSE.pos_x) * t;
        target_position.pos_y = start.pos_y + (stVecSE.pos_y) * t;

        return target_position;
    }


    // 路径执行阶段
    enum PATH_EXEC_STAGE
    {
        PATH_FIRST,
        PATH_RUN,
        PATH_END
    };

    // void sr_nav_line()
    // {
    //     double look_ahead_distance = 0.3;
    //     PathLine line_;
    //     PATH_EXEC_STAGE path_exec_state;

    //     ST_VECTOR stVecSE, stVecPE;
    //     float gama, remain_distance = 0;
        
    //     if (path_exec_state == PATH_FIRST)
    //     {
    //         double length = line_.getLineLength();
    //         if(length < 0.005f) path_exec_state = PATH_END;
    //         else                path_exec_state = PATH_RUN;
    //     }
    //     if (path_exec_state == PATH_RUN)
    //     {
    //         //根据机器人当前的速度确定前视距离的值。速度越小前视距离越小。前视距离不得小于0.3m
    //         LookAheadDistance = calc_lookahead_dist(pstNav, pstCtlPara);
    //         // 起点指向终点向量
    //         stVecSE.ssVx = pstNav->stPath.stPathPara->end_x - pstNav->stPath.stPathPara->start_x;
    //         stVecSE.ssVy = pstNav->stPath.stPathPara->end_y - pstNav->stPath.stPathPara->start_y;
    //         stVecSE.fpLength = cala_vector_dist(stVecSE);	//计算路径长度
    //         stVecSE.fpalpha = atan2(stVecSE.ssVy, stVecSE.ssVx);

    //         // 当前点指向终点向量
    //         stVecPE.ssVx = pstNav->stPath.stPathPara->end_x - pstRobot->stPot.ssPosX;
    //         stVecPE.ssVy = pstNav->stPath.stPathPara->end_y - pstRobot->stPot.ssPosY;
    //         stVecPE.fpalpha = atan2(stVecPE.ssVy, stVecPE.ssVx);
    //         // 计算纵向偏差
    //         pstNav->stNavPid.stPidVtc.fpE = CalRadialProjection(stVecPE, stVecSE);//计算剩余距离

    //         //规划速度
    //         if (pstNav->stPath.stPathPara->direction == 0x01)
    //         {
    //             pstNav->fpNavAngle = fabs(sr_convert_angle(pstRobot->stPot.ssPosQ - stVecPE.fpalpha));
    //         }
    //         else if (pstNav->stPath.stPathPara->direction == 0x02)
    //         {
    //             pstNav->fpNavAngle = fabs(fabs(sr_convert_angle(pstRobot->stPot.ssPosQ - stVecPE.fpalpha)) - PI);
    //         }

    //         //计算直线上的目标点。
    //         sr_pose_t start_pose, end_pose, current_pose;
    //         start_pose.pos_x = pstNav->stPath.stPathPara->start_x;
    //         start_pose.pos_y = pstNav->stPath.stPathPara->start_y;
    //         end_pose.pos_x = pstNav->stPath.stPathPara->end_x;
    //         end_pose.pos_y = pstNav->stPath.stPathPara->end_y;
    //         current_pose.pos_x = pstRobot->stPot.ssPosX;
    //         current_pose.pos_y = pstRobot->stPot.ssPosY;
    //         current_pose.pos_theta = pstRobot->stPot.ssPosQ;

    //         sr_pose_t target_pose = sr_calc_line_target_point(start_pose, end_pose, current_pose, LookAheadDistance);
    //         gama = sr_calc_coeff(current_pose, pstNav->stPath.stPathPara->direction, target_pose);
            
    //         sr_velocity_plan_t velocity_para;
    //         velocity_para.max_v = MIN(pstNav->stPath.stPathPara->fpMaxV, pstNav->stDynVelt.fpDLimitV);
    //         velocity_para.end_v = pstNav->stPath.stPathPara->end_v;
    //         velocity_para.acc_up = pstCtlPara->fpAccUpV;
    //         velocity_para.acc_down = pstCtlPara->fpAccDownV;
    //         velocity_para.remain_dist = pstNav->stNavPid.stPidVtc.fpE;

    //         float temp_v = 0.0f;
    //         temp_v = g_plan_v;//获取上一次规划速度

    //         //--wx--20220802--获取实际速度，同时修改上一次的规划速度，防止车辆控制异常时突然输出一个很大的速度
    //         get_act_v_and_mend_temp(&temp_v,g_c_beyond_v,pstCtlPara);

    //         //T型速度规划
    //         sr_velocity_control(velocity_para, 0.01f, &temp_v);
            
    //         //控制角速度
    //         float temp_w = temp_v * gama;
            
    //         if(pstNav->stPath.stPathPara->direction == 0x01)
    //         {
    //             pstNav->stVeltDes.fpV = temp_v;
    //             pstNav->stVeltDes.fpW = temp_w;
    //         }
    //         //倒车
    //         else if (pstNav->stPath.stPathPara->direction == 0x02)
    //         {
    //             pstNav->stVeltDes.fpV = -temp_v;
    //             pstNav->stVeltDes.fpW = temp_w;
    //         }

    //         //计算行走距离
    //         pstNav->fpCurDis = pstNav->stPath.stPathPara->fpDisFromStr - pstNav->stNavPid.stPidVtc.fpE;

    //         if (pstNav->stNavPid.stPidVtc.fpE < pstNav->stPotSen.ssPosY)
    //         {
    //             path_exec_state = PATH_END;
    //             //如果该路径的终点速度为0，则将目标速度设置为0
    //             if (pstNav->stPath.stPathPara->end_v == 0)
    //             {
    //                 pstNav->stVeltDes.fpV = 0;
    //             }
    //         }
    //     }

    //     if (path_exec_state == PATH_END)
    //     {

    //     }
    // }


    /**
     * 原地旋转函数
     * 
     * @param[in]
     * @param[out]
     *
     * @return
     */
    // void sr_nav_rotate(ST_ROBOT *pstRobot, ST_NAV *pstNav,sr_crl_para_t* pstCtlPara)
    // {
    //     // int i;
    //     float Rotfpalpha;
    //     static uint8_t Rotendflag = 0;
    //     static uint8_t AccDownFlag = 0;
    //     static float last_angle = 0;
    //     static float fpDeltaAngle; //已经转过的角度

    //     static uint32_t log_cnt_r = 0;
    //     float fpTempW = 0;		//临时角速度
        
    //     if (pstNav->stPath.emPathState == PATH_FIRST)
    //     {
    //         pstNav->stPath.emPathState = PATH_RUN;
    //         CalRotationPotPara(pstNav, pstRobot->stPot.ssPosQ); //计算原地旋转最大角速度
    //         pstNav->stPath.stPathPara->start_x = pstRobot->stPot.ssPosX;
    //         pstNav->stPath.stPathPara->start_y = pstRobot->stPot.ssPosY;
    //         pstNav->stPath.stPathPara->end_x = pstRobot->stPot.ssPosX;
    //         pstNav->stPath.stPathPara->end_y = pstRobot->stPot.ssPosY;
    //         pstNav->stNavPid.stPidTrvs.fpE = 0;
    //         pstNav->stNavPid.stPidVtc.fpE = 0;
            
    //         scurve_para_initial(); //S型曲线规划参数初始化

    //         pstNav->fpNavAngle = 0;
    //         //初始化时间信息
    //         pstNav->fpCtlt = 0;
            
    //         Rotendflag = 0; // 初始化

    //         last_angle = pstRobot->stPot.ssPosQ; //纪录初始角度为上一次角度
    //         fpDeltaAngle = 0; //清零转过角度

    //         if (fabs(pstNav->stPath.stRotation.fpRotAng) < 0.0087f) //当角度偏差小于±0.5度时-wx-0802之前写错了
    //         {
    //             pstNav->stPath.emPathState = PATH_END;
    //         }

    //         AccDownFlag = 0; 

    //         g_plan_v = pstNav->stVeltDes.fpV; //--wx-第一次进来，把上一次的给定速度给 规划速度，这样速度才能连续
    //     }

    //     //接收到direction為0x21表示到达终点后需要取货架。此时旋转目标角度就是作为终点角度记录下来。
    //     if (pstNav->stPath.stPathPara->direction == 0x21&&pstNav->stPath.stPathPara->empath_over==OVER_LAST)//放弃最后的原地旋转。
    //     {
    //         pstNav->stPath.emPathState = PATH_END;
    //         RotateStationDirFlag = 1;
    //     }

        
    //     if (pstNav->stPath.emPathState == PATH_RUN)
    //     {
    //         //只有当舵轮角度转到适当的位置时，才开始转动，角速度为v/b
    //         //其中b为舵轮中心到固定轮中心的距离值。
    //         //当舵轮角度为90度时，由于车体是围绕着固定轮中心旋转，所以车体转动半径为0.
            
    //         float Dangle = sr_convert_angle(pstRobot->stPot.ssPosQ - last_angle);

    //         fpDeltaAngle += Dangle;
            
    //         //转换到 0~pi ，当fpRotAng是 pi，目标又是pi的时候会出现 remain_angle无法小于零的情况，所以要加一道结束判断Rotendflag wx--20220624
    //         // fpDeltaAngle = fabs(sr_convert_angle(pstRobot->stPot.ssPosQ - pstNav->stPath.stPathPara->fpStartAngle));

    //         // pstNav->stNavPid.stPidVtc.fpE = (pstNav->stPath.stRotation.fpRotAng - fpDeltaAngle);

    //         float remain_angle = 0;
    //         if(pstNav->stPath.stRotation.fpRotAng < 0){
    //             remain_angle = -(pstNav->stPath.stRotation.fpRotAng - fpDeltaAngle); //剩余弧度
    //         }
    //         else{
    //             remain_angle = (pstNav->stPath.stRotation.fpRotAng - fpDeltaAngle); //剩余弧度
    //         }
                


    //         switch (g_speed_plan_mode)
    //         {
    //             case 0:
    //                 WConfigure_doubleS(&pstNav->stPath,(remain_angle),fabs(pstNav->stVeltDes.fpW),g_Scurve_plan_acc);
    //                 fpTempW = WNext_doubleS(&g_Scurve_plan_acc);
    //                 break;
    //             case 1:
    //             default:
    //             {
    //                 //T加减速
    //                 if (fabs(remain_angle - 0.04f) < (SQUARE(pstNav->stVeltDes.fpW)) / 2.0f / pstNav->stPath.stRotation.fpAccDownW)
    //                 {
    //                     //减速段 进入减速段不再从新进入加速
    //                     AccDownFlag = 1;
    //                 }
        

    //                 if (AccDownFlag == 0)
    //                 {
    //                     //加速
    //                     if (fabs(pstNav->stVeltDes.fpW) < pstNav->stPath.stRotation.fpMaxW)
    //                     {
    //                         //不到最高速，加
    //                         fpTempW = fabs(pstNav->stVeltDes.fpW) + pstNav->stPath.stRotation.fpAccUpW * 0.01f;
    //                     }
    //                     else{
    //                         //到了保持
    //                         fpTempW = pstNav->stPath.stRotation.fpMaxW;
    //                     }
    //                 }
    //                 else
    //                 {
    //                     fpTempW = fabs(pstNav->stVeltDes.fpW) - pstNav->stPath.stRotation.fpAccDownW * 0.01f;

    //                     if (fpTempW < 0.02)
    //                     {
    //                         fpTempW = 0.02;
    //                     }
    //                     //路径结束时，将AccDownFlag,AccDownCorrectFlag设置为0.
    //                 }
    //                 break;
    //             }
    //         }

    //         if (pstNav->stPath.stRotation.ucDirFlag == 0)
    //         { //反转
    //             fpTempW = -1.0f*fpTempW;
    //         }

    //         // 根据当前角度，目标角速度，旋转方向，计算出从当前角度根据旋转方向转到目标角度需要经过的角度值。这个值一直是正的
    //         Rotfpalpha = rotation_angle(pstRobot->stPot.ssPosQ,pstNav->stPath.stPathPara->fpEndAngle,pstNav->stPath.stRotation.ucDirFlag);

    //         g_plan_v = pstNav->stVeltDes.fpV;

        
    //         //更新控制时间
    //         pstNav->fpCtlt += 0.01f;
    //         //计算运行时间
    //         pstNav->fpCurtime = pstNav->stPath.stPathPara->fpTimeFromStr - pstNav->stPath.stPathPara->fpPathTime + pstNav->fpCtlt;

    //         if (Rotfpalpha < PI_8 && Rotendflag == 0)
    //         {
    //             Rotendflag = 1;
    //         }
    //         if (Rotfpalpha > PI3_2 && Rotendflag == 1)
    //         {
    //             //将目标速度设置为0
    //             //同时将目标角度设置为0，就是和车体保持一致。
    //             if (pstNav->emBaseType == GULF)
    //             {
    //                 fpTempW = 0;
    //                 Rotendflag = 2;
    //             }
    //             else
    //             {
    //                 pstNav->stPath.emPathState = PATH_END;
    //             }
    //         }

    //         if (remain_angle <= pstNav->stPotSen.ssPosQ &&
    //             pstNav->stPath.emPathState != PATH_END) //进入闭环区
    //         {
    //             if (pstNav->emBaseType == GULF)
    //             {
    //                 fpTempW = 0;
    //                 Rotendflag = 2; //旋转结束到位标志
    //             }
    //             else
    //             {
    //                 pstNav->stPath.emPathState = PATH_END;
    //             }
    //         }
    //     }

        
    //     if((Rotendflag == 2) && (pstNav->emBaseType == GULF))
    //     {
    //         fpTempW = 0;
    //         if(fabs(get_rotate_angle()) < 300){
    //             pstNav->stPath.emPathState = PATH_END;
    //         }
    //     }
        
        
    //     if (pstNav->stPath.emPathState == PATH_END)
    //     {
    //         fpTempW = 0;
    //         Rotendflag = 0;
    //         AccDownFlag = 0;
    //         // AccDownCorrectFlag = 0;
    //         fpDeltaAngle = 0;//清零转过角度
    //     }

        
    //     last_angle = pstRobot->stPot.ssPosQ;
    //     // // ---wx---20220707---最后把角度赋值出去，以免全局变量带来的问题
    //     pstNav->stVeltDes.fpW = fpTempW;
    //     pstNav->stVeltDes.fpV = 0; //原地旋转线速度为0
    // }
};

} // namespace motion

#endif