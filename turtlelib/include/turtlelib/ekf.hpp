#ifndef EKF_INCLUDE_GUARD_HPP
#define EKF_INCLUDE_GUARD_HPP
#include "turtlelib/diff_drive.hpp"
#include <armadillo>
#include <unordered_set>


namespace turtlelib
{
    constexpr int n = 15;
    class EKF
    {
    private:
        arma::vec state,state_est;
        arma::mat H, K;
        arma::mat cov, cov_est;
        RobotConfig prev_config, curr_config;
        std::unordered_set<int> obstacle_set{};
        int num_obstacles = 0;

    public:
        /// @brief Default constructor
        EKF();

        /// @brief Setter for the current robot config in the EKF class
        /// @param x - desired x coordinate
        /// @param y - desired y coordinate
        /// @param theta - desired theta
        /// @return - new desired robot config
        void setEKF_config(double x, double y, double theta);

        /// @brief get temp state vec
        /// @return Temp state
        arma::vec getState();

        /// @brief Generate A matrix for Gain Calculation
        /// @return A matrix
        arma::mat calc_Amat();

        /// @brief Calculate the predicted state and covariance
        void predict_state();



        /// @brief Correct the predicted state and covariance
        /// @param cx x coordinate of landmark
        /// @param cy y coordinate of landmark
        /// @param j index of landmark
        /// @return A robot config with the corrected q from the state vec
        void update_state(double cx, double cy, int j);

        /// @brief Perform data association 
        int data_association(double center_x, double center_y);
    };
}
#endif