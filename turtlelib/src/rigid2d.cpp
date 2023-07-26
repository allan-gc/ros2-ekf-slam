#include "turtlelib/rigid2d.hpp"
#include <cstdio>
#include <ostream>
#include <iostream>

/// \brief Defines functions that are included from rigid2d.hpp for 
/// rigid body transformations. Function documentations found in include file.

namespace turtlelib
{
    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        return os<<"["<< v.x <<" "<< v.y << "]";
    }

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        char c = is.peek();
        if (c == '['){
            is.get();
        }
        is >> v.x >> v.y;

        is.clear();
        is.ignore(100, '\n');
        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Twist2D & t)
    {
        return os<<"["<< t.w <<" "<< t.x << " " << t.y <<"]";
    }

    std::istream & operator>>(std::istream & is, Twist2D & t)
    {
        char c = is.peek();
        if (c == '['){
            is.get();
        }
        is >> t.w >> t.x >> t.y;

        is.clear();
        is.ignore(100, '\n');
        return is;

    }

    Transform2D::Transform2D(): vec{0.0,0.0}, rot{0.0} {}
    
    Transform2D::Transform2D(Vector2D trans): vec{trans.x, trans.y}, rot{0.0}{}

    Transform2D::Transform2D(double rad): vec{0.0, 0.0}, rot{rad}{}

    Transform2D::Transform2D(Vector2D trans, double rad): vec{trans.x, trans.y}, rot{rad}{}

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        Vector2D tf_vec;
        tf_vec.x=(cos(rot)*v.x)+(-sin(rot)*v.y)+vec.x; 
        tf_vec.y=(sin(rot)*v.x)+(cos(rot)*v.y)+vec.y;
        return tf_vec;
    }

    Transform2D Transform2D::inv() const
    {
        Transform2D inv_trans;

        inv_trans.vec.x= -vec.x*cos(rot)-vec.y*sin(rot);
        inv_trans.vec.y= -vec.y*cos(rot)+vec.x*sin(rot);
        inv_trans.rot=-rot;

        return inv_trans;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {   
        vec.x=(cos(rot)*rhs.vec.x)+(-sin(rot)*rhs.vec.y)+vec.x;
        vec.y=(sin(rot)*rhs.vec.x)+(cos(rot)*rhs.vec.y)+vec.y;
        rot=rot+rhs.rot;

        return *this;
    }

    Vector2D Transform2D::translation() const
    {
        Vector2D trans;
        trans.x = vec.x;
        trans.y = vec.y;
        return trans;
        //! Alternatively you can do
        //! return {vec.x, vec.y};
    }

    double Transform2D::rotation() const
    {
        double w = rot;
        return w;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
    {
        
        return os<< "deg: " <<rad2deg(tf.rot) << " x: " <<tf.vec.x << " y: "<<tf.vec.y;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf)
    {
        Vector2D vec_input;
        auto rot_input = 0.0;

        char c = is.peek();
        if (c == 'd'){
            char var1[]="deg:";
            char var2[]="x:";
            char var3[]="y:";

            is>>var1>>rot_input>>var2>>vec_input.x>>var3>>vec_input.y;
        }
        else
        {
            is >> rot_input >> vec_input.x >> vec_input.y;
        }

        is.clear();
        is.ignore(100, '\n');
        tf=Transform2D(vec_input,(deg2rad(rot_input)));

        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        return lhs*=rhs;
    }

    Twist2D Transform2D::conv_frame(const Twist2D t1)
    {
        Twist2D t2;

        t2.w = t1.w;
        t2.x = t1.w*vec.y + t1.x*cos(rot) - t1.y*sin(rot);
        t2.y = -vec.x*t1.w + t1.x*sin(rot) + t1.y*cos(rot);

        return t2;
    }

    Transform2D integrate_twist(Twist2D t1)
    {
        Transform2D Tsb, Tss;
        Vector2D vec, vec1;
        double rot, rot1;
        if (almost_equal(t1.w,0.0))
        {
            rot = 0.0;
            vec.x= t1.x;
            vec.y= t1.y;
            Tsb= Transform2D(vec,rot);
            return Tsb;
        }
        else
        {
            rot = 0.0;
            vec.x= t1.y/t1.w;
            vec.y= -t1.x/t1.w;

            rot1=t1.w;
            vec1.x=0.0;
            vec1.y=0.0;

            Tsb= Transform2D{{vec.x,vec.y},rot};
            Tss= Transform2D{{vec1.x,vec1.y},rot1};
            return Tsb.inv()*Tss*Tsb;
        }
    }

    Vector2D norm(const Vector2D v)
    {
        Vector2D norm_vec;
        auto mag = sqrt(pow(v.x,2) + pow(v.y,2));

        norm_vec.x = v.x/mag;
        norm_vec.y = v.y/mag;

        return norm_vec;
    }

    Vector2D & Vector2D::operator+=(const Vector2D & rhs)
    {
        x=x+rhs.x;
        y=y+rhs.y;

        return *this;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D & rhs)
    {
        return lhs+=rhs;
    }

     Vector2D & Vector2D::operator-=(const Vector2D & rhs)
    {
        x=x-rhs.x;
        y=y-rhs.y;

        return *this;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D & rhs)
    {
        return lhs-=rhs;
    }


    Vector2D & Vector2D::operator*=(const double scalar)
    {
        x=x*scalar;
        y=y*scalar;

        return *this;
    }

    Vector2D operator*(Vector2D vec, const double scalar)
    {
        return vec*=scalar;
    }

    Vector2D operator*(const double scalar, Vector2D vec)
    {
        return vec*=scalar;
    }

    double dot(Vector2D & vec1, Vector2D & vec2)
    {
        return (vec1.x*vec2.x)+(vec1.y*vec2.y);
    }

    double magnitude(Vector2D & vec)
    {
        return sqrt(pow(vec.x,2) + pow(vec.y,2));
    }

    double angle(Vector2D & vec1, Vector2D & vec2)
    {
        return atan2((vec1.x*vec2.y- vec1.y*vec2.x), (vec1.x*vec2.x + vec1.y*vec2.y));
    }

    double calcDistance(double x1, double y1, double x2, double y2)
    {
        return sqrt(pow(x2-x1,2) + pow(y2-y1,2));
    }

}


