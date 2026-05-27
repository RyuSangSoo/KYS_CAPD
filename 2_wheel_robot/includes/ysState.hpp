#ifndef YSSTATE_HPP
#define YSSTATE_HPP

#include "header.hpp"

typedef struct
{
    // Ref: 제어기가 만들고 추종하려는 목표 상태
    // Est: 센서/odometry 기반으로 추정한 현재 상태
    struct
    {
        struct
        {
            struct
            {
                // 바디 기준 상태
                // x, y, yaw는 월드 좌표계 기준 포즈 또는 속도를 저장한다.
                double x        = 0.0;
                double y        = 0.0;
                double yaw      = 0.0;

            }Base;

            struct
            {
                // 좌/우 바퀴 상태
                // Pos: 바퀴 각도
                // Vel: 바퀴 각속도
                // Tau: 바퀴 토크(또는 effort command)
                double left     = 0.0;
                double right    = 0.0;

            }Wheel;
            
        }Pos, Vel, Tau;

    }Ref, Est;
    
}Robot;

extern Robot robot;

#endif
