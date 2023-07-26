#include <cstdio>
#include <ostream>
#include <iostream>
#include "turtlelib/diff_drive.hpp"

namespace turtlelib

{
    DiffDrive::DiffDrive()
    : wheel_radius{rad}, 
      track_length{track},
      wheel_speeds{0.0,0.0}, 
      config{0.0,0.0,0.0} {}

    DiffDrive::DiffDrive(double radii, double track_width)
    : wheel_radius{radii}, 
      track_length{track_width},
      wheel_speeds{0.0,0.0}, 
      config{0.0,0.0,0.0} {}

    DiffDrive::DiffDrive(double radii, double track_width, WheelCmds speeds, RobotConfig position)
    : wheel_radius{radii},
      track_length{track_width}, 
      wheel_speeds{speeds}, 
      config{position} {}

    RobotConfig DiffDrive::configuration() const
    {
        RobotConfig curr_config;
        curr_config.theta = config.theta;
        curr_config.x = config.x;
        curr_config.y = config.y;

        return curr_config;
    }

    void DiffDrive::setConfig(double x, double y, double theta)
    {
      config.x = x;
      config.y = y;
      config.theta = theta;
    }

    Twist2D DiffDrive::getBodyTwist(WheelCmds new_wheel_pos)
    {
      double d = track_length/2;
      double w = (wheel_radius/2) * (new_wheel_pos.r_wheel - new_wheel_pos.l_wheel)/d;
   
      double x = (wheel_radius/2) *(new_wheel_pos.r_wheel + new_wheel_pos.l_wheel);
      double y = 0.0;
      return {w, x, y};
    }

    WheelCmds DiffDrive::inverse_kinematics(const Twist2D twist)
    {
        WheelCmds wheel_speeds;
        double d = track_length/2;

        if (twist.y != 0.0)
        {
            throw (std::logic_error("Non-zero y component causes slippage of wheels, value must be zero"));
        }
        wheel_speeds.r_wheel = ((-d/wheel_radius) * twist.w) + ((1/wheel_radius)*twist.x);
        wheel_speeds.l_wheel = ((d/wheel_radius) * twist.w) + ((1/wheel_radius)*twist.x);
        return wheel_speeds;
    }

    void DiffDrive::forward_kinematics(WheelCmds new_wheel_pos)
    {
        Twist2D delta_q, Vb;
        Transform2D Tbb_prime;

        Vb = getBodyTwist(new_wheel_pos);

        Tbb_prime = integrate_twist(Vb);
        delta_q.w = Tbb_prime.rotation();
        delta_q.x = (cos(config.theta)*Tbb_prime.translation().x) - (sin(config.theta)*Tbb_prime.translation().y);
        delta_q.y = (sin(config.theta)*Tbb_prime.translation().x) + (cos(config.theta)*Tbb_prime.translation().y);

        config.theta = config.theta + delta_q.w;
        config.x = config.x + delta_q.x;
        config.y = config.y + delta_q.y;

    }
}