#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <cstdio>
#include <ostream>
#include <iostream>

/// \brief Uses the functions from rigid2d to perform
/// 2D body transformations based off of user input.
/// Prompts user for transformations, vectors, and twists
/// in different reference frames

int main(void) {

    turtlelib::Vector2D vec_b, vec_bhat, vec_a, vec_c;
    turtlelib::Twist2D V_a, V_b, V_c;
    turtlelib::Transform2D T_ab, T_bc, T_ba, T_cb, T_ac, T_ca, test1;

    printf("Enter transform T_{a,b}: \n");
    std::cin>> T_ab;

    printf("Enter transform T_{b,c}: \n");
    std::cin>> T_bc;

    printf("T_{a,b}: ");
    std::cout<< T_ab <<std::endl;

    T_ba= T_ab.inv();
    printf("T_{b,a}: ");
    std::cout<< T_ba <<std::endl;

    printf("T_{b,c}: ");
    std::cout<< T_bc <<std::endl;

    T_cb= T_bc.inv();
    printf("T_{c,b}: ");
    std::cout<< T_cb <<std::endl;

    T_ac=T_ab*T_bc;
    printf("T_{a,c}: ");
    std::cout<< T_ac <<std::endl;

    T_ca= T_ac.inv();
    printf("T_{c,a}: ");
    std::cout<< T_ca <<std::endl;

    printf("Enter vector v_b: \n");
    std::cin>> vec_b;

    vec_bhat = turtlelib::norm(vec_b);
    printf("v_bhat: ");
    std::cout<< vec_bhat <<std::endl;

    vec_a = T_ab(vec_b);
    printf("v_a: ");
    std::cout<< vec_a <<std::endl;

    printf("v_b: ");
    std::cout<< vec_b <<std::endl;

    vec_c = T_cb(vec_b);
    printf("v_c: ");
    std::cout<< vec_c <<std::endl;

    printf("Enter twist V_b: \n");
    std::cin>> V_b;

    V_a = T_ab.conv_frame(V_b);
    printf("v_a: ");
    std::cout<< V_a <<std::endl;

    printf("V_b: ");
    std::cout<< V_b <<std::endl;

    V_c = T_cb.conv_frame(V_b);
    printf("V_c: ");
    std::cout<< V_c <<std::endl;

    return 0;
}