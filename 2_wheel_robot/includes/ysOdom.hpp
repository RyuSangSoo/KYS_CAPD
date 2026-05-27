#ifndef YSODOM_HPP
#define YSODOM_HPP

#include "header.hpp"
#include "ysState.hpp"

class ysOdom
{
public:
    ysOdom(double wheel_radius, double wheel_base);

    void Update(double dt);
    void Reset();

    double GetWheelRadius() const;
    double GetWheelBase() const;

private:
    double NormalizeAngle(double angle) const;

    double wheel_radius_;
    double wheel_base_;
};

#endif
