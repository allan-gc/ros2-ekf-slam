#ifndef CIRCLE_FIT_INCLUDE_GUARD_HPP
#define CIRCLE_FIT_INCLUDE_GUARD_HPP
/// \file
/// \brief Circle fit class
#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>
#include "turtlelib/rigid2d.hpp"
#include <vector>

namespace turtlelib
{

    struct Circle
    {
        Vector2D center;
        double radius;
    };

    Circle circle_fit(std::vector<Vector2D> cluster);

}


#endif