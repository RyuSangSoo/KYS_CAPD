#include "ysOdom.hpp"

ysOdom::ysOdom(double wheel_radius, double wheel_base)
: wheel_radius_(wheel_radius), wheel_base_(wheel_base)
{
}

void ysOdom::Update(double dt)
{
    const double wl = robot.Est.Vel.Wheel.left;
    const double wr = robot.Est.Vel.Wheel.right;

    robot.Est.Vel.Base.x = wheel_radius_ * 0.5 * (wr + wl);
    robot.Est.Vel.Base.y = 0.0;
    robot.Est.Vel.Base.yaw = wheel_radius_ * (wr - wl) / wheel_base_;

    const double yaw = robot.Est.Pos.Base.yaw;
    robot.Est.Pos.Base.x += robot.Est.Vel.Base.x * std::cos(yaw) * dt;
    robot.Est.Pos.Base.y += robot.Est.Vel.Base.x * std::sin(yaw) * dt;
    robot.Est.Pos.Base.yaw += robot.Est.Vel.Base.yaw * dt;
    robot.Est.Pos.Base.yaw = NormalizeAngle(robot.Est.Pos.Base.yaw);
}

void ysOdom::Reset()
{
    robot.Est.Pos.Base.x = 0.0;
    robot.Est.Pos.Base.y = 0.0;
    robot.Est.Pos.Base.yaw = 0.0;

    robot.Est.Vel.Base.x = 0.0;
    robot.Est.Vel.Base.y = 0.0;
    robot.Est.Vel.Base.yaw = 0.0;
}

double ysOdom::GetWheelRadius() const
{
    return wheel_radius_;
}

double ysOdom::GetWheelBase() const
{
    return wheel_base_;
}

double ysOdom::NormalizeAngle(double angle) const
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }

    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }

    return angle;
}
