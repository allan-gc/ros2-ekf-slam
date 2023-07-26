#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Diff Drive class
#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>
#include "turtlelib/rigid2d.hpp"


namespace turtlelib
{   

    /// @brief Base wheel radius and track width for robot from current yaml file
    constexpr double rad = 0.033;
    constexpr double track = 0.16;

    /// @brief Struct for wheel speeds of diff drive robot
    struct WheelCmds
    {
        double r_wheel;
        double l_wheel;
    };

    /// @brief The configuration of diff drive robot
    struct RobotConfig
    {
        /// @brief angle of robot chassis
        double theta;

        /// @brief x and y coordinate of robot chassis
        double x;
        double y;
    };


    /// @brief A differential drive robot class that defines its configuration and control
    class DiffDrive
    {
    private:
        double wheel_radius;
        double track_length;
        WheelCmds wheel_speeds;
        RobotConfig config;

    public:

        /// @brief create robot with wheel radius and track width from param file
        DiffDrive();

        /// @brief create robot with zero wheel speeds and initial zero position
        /// @param radii - the radius of the wheels
        /// @param track_width - distance between the wheels
        DiffDrive(double radii, double track_width);

        /// @brief create robot with all params initialized
        /// @param radii - the radius of the wheels
        /// @param track_width - distance between the wheels
        /// @param position - the start position of the robot
        /// @param speeds - the start speeds of the wheels
        DiffDrive(double radii, double track_width, WheelCmds speeds, RobotConfig position);

        /// @brief the current configuration of the robot
        /// @return the robot config struct with theta,x, and y
        RobotConfig configuration() const;


        /// @brief Setter for robot config
        /// @param x - desired x coordinate
        /// @param y - desired y coordinate
        /// @param theta - desired theta
        /// @return - new desired robot config
        void setConfig(double x, double y, double theta);

        /// @brief Calculates the inverse kinematics of the robot using a given twist. See Eq. 1-2 in /doc for derivation
        /// @param twist - the given twist to follow
        /// @return - the wheel velocities needed to make the robot follow the twist
        WheelCmds inverse_kinematics(const Twist2D twist);

        /// @brief Calculates the forward kinematics of the robot given new wheel positions. See Eq 4 in /doc for derivation
        /// @param new_wheel_pos - the new wheel positions
        /// This functions updates the current RobotConfig of the class by adding the change in configuration that 
        /// the Fk function calculates. It assumes that the new inputted wheel positions is the change in wheel positions that you want
        /// the robot to move by , not the wheel position that the robot should end up at. 
        void forward_kinematics(WheelCmds new_wheel_pos);

        /// @brief create body twist from wheel commands. See Eq.3 in /doc
        /// @param new_wheel_pos - inputted wheel position commands
        /// @return - the body twist Vb
        Twist2D getBodyTwist(WheelCmds new_wheel_pos);
       
    };
}

#endif