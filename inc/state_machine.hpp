#ifndef _STATE_MACHINE_HPP_
#define _STATE_MACHINE_HPP_

namespace motion{

typedef enum
{
    MODE_IDLE,  // 空闲
    MODE_MANUAL,// 手动
    MODE_AUTO,  // 自动
    MODE_TRACK  // 跟踪
}MC_MODE;

typedef enum
{
    STAT_RUN,   // 运行
    STAT_PAUSE, // 暂停
    STAT_STOP   // 停止
}MC_STAT;

typedef enum
{
    MCMD_NULL  = 0x00,   // 空指令
    MCMD_RUN   = 0x01,   // 运行指令
    MCMD_STOP  = 0x02,   // 停止指令
    MCMD_PAUSE = 0x03,   // 暂停指令
    //MCMD_CONTI = 0x04    // 继续指令
}MC_MCMD;

enum FSM_STATE
{
    SM_IDLE,//STATE 空闲
    SM_MANUAL,      // 手动
    SM_AUTO_PATH,   // 自动路径导航
    SM_RUN,         // 运行
    SM_PAUSE,       // 暂停
    SM_CONTINUE,    // 继续
    SM_STOP,        // 停止
    SM_TRACK
};

enum CMD_TYPE
{
    CMD_RUN = 0x01,             // 运行指令
    CMD_STOP = 0x02,            // 停止指令
    CMD_PAUSE = 0x03,           // 暂停指令
    CMD_CONTINUE = 0x04,        // 继续指令

    CMD_PATH_MODE = 0x05, 	    // 进入执行路径模式
    CMD_VELOCITY_MODE = 0x06,   // 进入执行速度模式
};

class StateMachine
{
private:
    FSM_STATE cur_state_;
    // CMD_TYPE cmd_;

public:
    StateMachine(FSM_STATE state = SM_IDLE)
        : cur_state_(state){

    };
    ~StateMachine(){};

    CMD_TYPE cmd_;
    void getCmd()
    {

    }

    FSM_STATE getState() const{
        return cur_state_;
    }

    void init()
    {
        cur_state_ = SM_IDLE;
    }

    void update()
    {
        switch(cur_state_)
        {
            case SM_IDLE:
            {
                switch(cmd_)
                {
                    case CMD_VELOCITY_MODE:cur_state_ = SM_MANUAL;break;
                    case CMD_PATH_MODE:cur_state_ = SM_AUTO_PATH;break;
                    default:break;
                }
            }break;
            case SM_MANUAL:
            {		
                switch(cmd_)
                {
                    case CMD_STOP:cur_state_ = SM_STOP;break;
                    case CMD_PAUSE:cur_state_ = SM_IDLE;break;
                    default:break;
                }
            }break;	
            case SM_AUTO_PATH:
            {
                switch(cmd_)
                {
                    case CMD_RUN:  cur_state_ = SM_RUN;break;
                    case CMD_STOP: cur_state_ = SM_STOP;break;
                    default:break;
                }
            }break;
            case SM_RUN:
            {
                switch(cmd_)
                {
                    case CMD_PAUSE:cur_state_ = SM_PAUSE;break;
                    case CMD_STOP:cur_state_ = SM_STOP;break;
                    default:break;
                }				
            }break;
            case SM_PAUSE:
            {
                switch(cmd_)
                {
                    case CMD_CONTINUE:cur_state_ = SM_CONTINUE;break;
                    case CMD_STOP:cur_state_ = SM_STOP;break;
                    default:break;
                }
            }break;
            case SM_CONTINUE:cur_state_ = SM_RUN;break;
            case SM_STOP:cur_state_ = SM_IDLE;break;
            default:break;//此处必须做处理，例如将速度设置为0等，防止意外			
        }
    }
};

typedef std::shared_ptr<StateMachine> StateMachine_ptr;

} // namespace motion

#endif