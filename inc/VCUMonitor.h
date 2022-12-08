#pragma once

//VCU状态监视接口

struct VCUMonitor
{
    virtual void onGearChanged(unsigned char /*active*/, unsigned char /*gear*/, unsigned char /*errCode*/){}
    virtual void onSteeringChanged(unsigned char /*active*/, float /*steer*/, unsigned char /*errCode*/){}
    virtual void onDriveChanged(unsigned char /*active*/, float /*speed*/, unsigned char /*errCode*/){}
    virtual void onBrakingChanged(unsigned char /*active*/, float /*brakeOpening*/, unsigned char /*brakeSta*/, unsigned char /*errCode*/){}
    virtual void onPakingChanged(unsigned char /*active*/, unsigned char /*pake*/, unsigned char /*errCode*/){}
    virtual void onOdometerChanged(unsigned char /*active*/, int /*posX*/, int /*posY*/, float /*heading*/){}
    virtual void onWheelSpdChanged(unsigned char /*active*/, float /*leftSpd*/, float /*rightSpd*/){}
    virtual void onWheelPulseChanged(unsigned char /*active*/, unsigned char /*leftPluse*/, unsigned char /*rightPluse*/){}
    virtual void onMileageAndBodyChanged(unsigned char /*active*/, float /*totalMileage*/, unsigned char /*frontCollide*/, unsigned char /*backCollide*/){}
};
