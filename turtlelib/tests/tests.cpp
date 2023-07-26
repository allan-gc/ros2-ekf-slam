#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/circle_fit.hpp"
#include <sstream>

using turtlelib::Transform2D;

TEST_CASE("Identity Transformation", "[transform]") // Garcia, Allan
{
    turtlelib::Vector2D vec;
    vec.x = 0.0;
    vec.y = 0.0;
    double rot = 0.0;

    turtlelib::Transform2D tf = turtlelib::Transform2D();
    REQUIRE(tf.translation().x == vec.x);
    REQUIRE(tf.translation().y == vec.y);
    REQUIRE(tf.rotation() == rot);
}

TEST_CASE("Rotation","[transform]") // Garcia, Allan
{
    double rot = 90;

    turtlelib::Transform2D tf = turtlelib::Transform2D(turtlelib::deg2rad(rot));
    REQUIRE(tf.rotation()== turtlelib::deg2rad(rot));
}


TEST_CASE("Change Twist frame","[transform]") // Garcia, Allan
{
    turtlelib::Twist2D v_b, v_a;
    v_b.w = 1;
    v_b.x = 1;
    v_b.y = 1;

    turtlelib::Vector2D vec;
    vec.x = 0.0;
    vec.y = 1.0;
    double rot = turtlelib::deg2rad(90);

    turtlelib::Transform2D t_ab{vec,rot}; //= turtlelib::Transform2D(vec,rot);
    v_a=t_ab.conv_frame(v_b);
    
    REQUIRE(v_a.w == 1.0);
    REQUIRE(v_a.x == 0.0);
    REQUIRE(v_a.y == 1.0);
}

TEST_CASE(" * Operator for tf multiplication","[transform]") // Garcia, Allan
{
    turtlelib::Vector2D vec1, vec2;
    vec1.x = 0.0;
    vec1.y = 1.0;

    vec2.x = 1.0;
    vec2.y = 0.0;
    double rot = turtlelib::deg2rad(90);

    turtlelib::Transform2D t_ab, t_bc, t_ac;
    t_ab = turtlelib::Transform2D(vec1,rot);
    t_bc = turtlelib::Transform2D(vec2,rot);
    
    t_ac=t_ab*t_bc;

    REQUIRE(turtlelib::almost_equal(t_ac.rotation(), turtlelib::deg2rad(180)));
    REQUIRE(turtlelib::almost_equal(t_ac.translation().x, 6.12323e-17));
    REQUIRE(turtlelib::almost_equal(t_ac.translation().y, 2.0));

}

TEST_CASE( "Operator () for Vector2D", "[transform]" ) { // Yin, Hang
   float my_x = 2;
   float my_y = 3;
   float my_ang = turtlelib::deg2rad(180);
   turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
   turtlelib::Vector2D v = {2,2};
   turtlelib::Vector2D result = Ttest(v);
   REQUIRE( turtlelib::almost_equal(result.x, 0.0, 1.0e-5) );
   REQUIRE( turtlelib::almost_equal(result.y, 1.0, 1.0e-5) );
}

TEST_CASE("operator()(Twist2D t)","[transform]") // Marno, Nel
{
   double test_rot = turtlelib::deg2rad(90);
   double test_x = 0.0;
   double test_y = 1.0;
   turtlelib::Transform2D T_ab{{test_x,test_y}, test_rot};
   turtlelib::Twist2D V_b{1, 1, 1};
   turtlelib::Twist2D V_a = T_ab.conv_frame(V_b); 
   REQUIRE(turtlelib::almost_equal(V_a.w, 1.0));
   REQUIRE(turtlelib::almost_equal(V_a.x, 0.0));
   REQUIRE(turtlelib::almost_equal(V_a.y, 1.0));
}

TEST_CASE( "Stream extraction operator >>", "[transform]" ) // Ava, Zahedi
{
   turtlelib::Transform2D tf = turtlelib::Transform2D();
   std::stringstream sstr;
   sstr << "deg: 90 x: 1 y: 3.4";
   sstr >> tf;
   REQUIRE( turtlelib::almost_equal(tf.rotation(), turtlelib::deg2rad(90), 0.00001) );
   REQUIRE( turtlelib::almost_equal(tf.translation().x, 1.0, 0.00001) );
   REQUIRE( turtlelib::almost_equal(tf.translation().y, 3.4, 0.00001) );
}

TEST_CASE("inv()", "[transform]"){ // Liz, Metzger
   turtlelib::Vector2D trans_vec = {0.0, 1.0};
   Transform2D test_trans = {trans_vec, turtlelib::deg2rad(90)};
   Transform2D test_inv = test_trans.inv();
   REQUIRE(turtlelib::almost_equal(test_inv.translation().x, -1.0));
   REQUIRE(turtlelib::almost_equal(test_inv.translation().y, -6.12323e-17));
   REQUIRE(turtlelib::almost_equal(test_inv.rotation(), -test_trans.rotation()));
   }

TEST_CASE("normalize", "[transform]"){ 
   double pi = turtlelib::PI;
   REQUIRE_THAT( turtlelib::normalize_angle(-5*pi/2), Catch::Matchers::WithinAbs(-pi/2, 1e-8));
   }

TEST_CASE("integrate twist for translation", "[transform]"){ 
   turtlelib::Twist2D t1;
   t1.w=0;
   t1.x= 2.5;
   t1.y= 1.2;
   turtlelib::Transform2D res;

   res=turtlelib::integrate_twist(t1);
   REQUIRE_THAT( res.translation().x, Catch::Matchers::WithinAbs(t1.x, 1e-8));
   REQUIRE_THAT( res.translation().y, Catch::Matchers::WithinAbs(t1.y, 1e-8));
}

TEST_CASE("integrate twist for rotation", "[transform]"){ 
   turtlelib::Twist2D t1;
   t1.w=1.5;
   t1.x= 0.0;
   t1.y= 0.0;

   turtlelib::Transform2D res;
   res=turtlelib::integrate_twist(t1);

   REQUIRE_THAT(res.rotation(), Catch::Matchers::WithinAbs(t1.w, 1e-8));
}

TEST_CASE("integrate twist for rotation + translation", "[transform]"){ 
   turtlelib::Twist2D t1;
   t1.w=1.5;
   t1.x= 2.65;;
   t1.y= -0.57;

   turtlelib::Transform2D res;
   res=turtlelib::integrate_twist(t1);

   REQUIRE_THAT(res.rotation(), Catch::Matchers::WithinAbs(t1.w, 1e-8));
   REQUIRE_THAT( res.translation().x, Catch::Matchers::WithinAbs(2.11536, 1e-5));
   REQUIRE_THAT( res.translation().y, Catch::Matchers::WithinAbs(1.26265, 1e-5));
}

TEST_CASE("FK Drive Forward", "[diff_drive]") {
    turtlelib::WheelCmds wheel_speeds;
    wheel_speeds.r_wheel = 1.45;
    wheel_speeds.l_wheel = 1.45;
    double radius = 1.0;
    double width = 4.0;
    turtlelib::DiffDrive robot = turtlelib::DiffDrive(radius, width);
    robot.forward_kinematics(wheel_speeds);
    REQUIRE_THAT(robot.configuration().theta, Catch::Matchers::WithinAbs(0.0 , 1.0e-5));
    REQUIRE_THAT(robot.configuration().x, Catch::Matchers::WithinAbs(1.45, 1.0e-5));
    REQUIRE_THAT(robot.configuration().y, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
}

TEST_CASE("IK Drive Forward", "[diff_drive]") {
    turtlelib::Twist2D V_a{0.0,2.5,0.0};
    double radius = 1.0;
    double width = 4.0;
    turtlelib::DiffDrive robot = turtlelib::DiffDrive(radius, width);
    turtlelib::WheelCmds wheel_speeds = robot.inverse_kinematics(V_a);
    REQUIRE_THAT(wheel_speeds.r_wheel, Catch::Matchers::WithinAbs(2.5 , 1.0e-5));
    REQUIRE_THAT(wheel_speeds.l_wheel, Catch::Matchers::WithinAbs(2.5, 1.0e-5));
}

TEST_CASE("FK Rotation", "[diff_drive]") {
    turtlelib::WheelCmds wheel_speeds;
    wheel_speeds.r_wheel = -3.218;
    wheel_speeds.l_wheel = 3.218;
    double radius = 1.0;
    double width = 4.0;
    turtlelib::DiffDrive robot = turtlelib::DiffDrive(radius, width);
    robot.forward_kinematics(wheel_speeds);
    REQUIRE_THAT(robot.configuration().theta, Catch::Matchers::WithinAbs(-1.609 , 1.0e-5));
    REQUIRE_THAT(robot.configuration().x, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(robot.configuration().y, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
}

TEST_CASE("IK Rotation", "[diff_drive]") {
    turtlelib::Twist2D V_a{-0.571,0.0,0.0};
    double radius = 1.0;
    double width = 4.0;
    turtlelib::DiffDrive robot = turtlelib::DiffDrive(radius, width);
    turtlelib::WheelCmds wheel_speeds = robot.inverse_kinematics(V_a);
    REQUIRE_THAT(wheel_speeds.r_wheel, Catch::Matchers::WithinAbs(1.142 , 1.0e-5));
    REQUIRE_THAT(wheel_speeds.l_wheel, Catch::Matchers::WithinAbs(-1.142, 1.0e-5));
}

TEST_CASE("FK Following Arc", "[diff_drive]") {
    turtlelib::WheelCmds wheel_speeds;
    wheel_speeds.r_wheel = 1.2654;
    wheel_speeds.l_wheel = 0.9547;
    double radius = 1.0;
    double width = 4.0;
    turtlelib::DiffDrive robot = turtlelib::DiffDrive(radius, width);
    robot.forward_kinematics(wheel_speeds);
    REQUIRE_THAT(robot.configuration().theta, Catch::Matchers::WithinAbs(0.077675, 1.0e-5));
    REQUIRE_THAT(robot.configuration().x, Catch::Matchers::WithinAbs(1.10893, 1.0e-5));
    REQUIRE_THAT(robot.configuration().y, Catch::Matchers::WithinAbs(0.0430899, 1.0e-5));
}

TEST_CASE("IK Following Arc", "[diff_drive]") {
    turtlelib::Twist2D V_a{0.64,1.235,0.0};
    double radius = 1.0;
    double width = 4.0;
    turtlelib::DiffDrive robot = turtlelib::DiffDrive(radius, width);
    turtlelib::WheelCmds wheel_speeds = robot.inverse_kinematics(V_a);
    REQUIRE_THAT(wheel_speeds.r_wheel, Catch::Matchers::WithinAbs(-0.045 , 1.0e-5));
    REQUIRE_THAT(wheel_speeds.l_wheel, Catch::Matchers::WithinAbs(2.515, 1.0e-5));
}


TEST_CASE("Circle Test 1, [circle_fit]") {

    std::vector<turtlelib::Vector2D> single_cluster;

    single_cluster.push_back(turtlelib::Vector2D{1,7});
    single_cluster.push_back(turtlelib::Vector2D{2,6});
    single_cluster.push_back(turtlelib::Vector2D{5,8});
    single_cluster.push_back(turtlelib::Vector2D{7,7});
    single_cluster.push_back(turtlelib::Vector2D{9,5});
    single_cluster.push_back(turtlelib::Vector2D{3,7});

    turtlelib::Circle circle = turtlelib::circle_fit(single_cluster);

    REQUIRE_THAT(circle.center.x, Catch::Matchers::WithinAbs(4.615482 , 1.0e-4));
    REQUIRE_THAT(circle.center.y, Catch::Matchers::WithinAbs(2.807354, 1.0e-4));
    REQUIRE_THAT(circle.radius, Catch::Matchers::WithinAbs(4.8275, 1.0e-4));
}

TEST_CASE("Circle Test 2, [circle_fit]") {

    std::vector<turtlelib::Vector2D> single_cluster;

    single_cluster.push_back(turtlelib::Vector2D{-1,0});
    single_cluster.push_back(turtlelib::Vector2D{-0.3,-0.06});
    single_cluster.push_back(turtlelib::Vector2D{0.3,0.1});
    single_cluster.push_back(turtlelib::Vector2D{1,0});
 

    turtlelib::Circle circle = turtlelib::circle_fit(single_cluster);

    REQUIRE_THAT(circle.center.x, Catch::Matchers::WithinAbs(0.4908357, 1.0e-4));
    REQUIRE_THAT(circle.center.y, Catch::Matchers::WithinAbs(-22.15212, 1.0e-4));
    REQUIRE_THAT(circle.radius, Catch::Matchers::WithinAbs(22.17979, 1.0e-4));
}


